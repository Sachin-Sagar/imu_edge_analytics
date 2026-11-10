#!/usr/bin/env python3
"""
Notebook Execution & Output Populator:
Executes each notebook cell in order, capturing stdout text streams and Matplotlib figures
as base64 inline images, and saves fully rendered .ipynb files.
This ensures that when opened in VS Code or any notebook viewer, all figures, tables,
and outputs are already rendered without requiring manual execution.
"""
import os
import sys
import json
import io
import base64
os.environ['MPLCONFIGDIR'] = '/tmp'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def execute_notebook(nb_path):
    print(f"Executing and populating: {nb_path}")
    with open(nb_path, 'r', encoding='utf-8') as f:
        nb = json.load(f)

    # Isolated execution environment for the notebook
    env = {
        '__name__': '__main__',
        '__file__': os.path.abspath(nb_path)
    }

    # Custom show hook to capture matplotlib figures
    captured_figures = []

    def custom_show(*args, **kwargs):
        figs = [plt.figure(i) for i in plt.get_fignums()]
        for fig in figs:
            buf = io.BytesIO()
            fig.savefig(buf, format='png', dpi=120, bbox_inches='tight')
            buf.seek(0)
            img_b64 = base64.b64encode(buf.read()).decode('utf-8')
            captured_figures.append(img_b64)
            plt.close(fig)

    exec_counter = 1

    for cell in nb['cells']:
        if cell['cell_type'] == 'code':
            code = ''.join(cell['source'])
            # Filter out IPython line magics (e.g. %matplotlib inline) for standard exec()
            code_lines = [l for l in code.splitlines(keepends=True) if not l.strip().startswith('%')]
            code_exec = ''.join(code_lines)
            
            # Setup stdout capture
            old_stdout = sys.stdout
            captured_stdout = io.StringIO()
            sys.stdout = captured_stdout
            
            # Monkey-patch plt.show
            captured_figures.clear()
            plt.show = custom_show

            outputs = []
            try:
                exec(code_exec, env)
                
                # Check for unclosed figures if plt.show() was omitted or called
                figs = [plt.figure(i) for i in plt.get_fignums()]
                for fig in figs:
                    buf = io.BytesIO()
                    fig.savefig(buf, format='png', dpi=120, bbox_inches='tight')
                    buf.seek(0)
                    img_b64 = base64.b64encode(buf.read()).decode('utf-8')
                    captured_figures.append(img_b64)
                    plt.close(fig)

            except Exception as e:
                print(f"Error executing cell in {nb_path}: {e}", file=old_stdout)
                import traceback
                traceback.print_exc(file=old_stdout)
            finally:
                sys.stdout = old_stdout

            # Process stdout stream
            stdout_text = captured_stdout.getvalue()
            if stdout_text:
                outputs.append({
                    'output_type': 'stream',
                    'name': 'stdout',
                    'text': stdout_text.splitlines(keepends=True)
                })

            # Process inline display images
            for img_b64 in captured_figures:
                outputs.append({
                    'output_type': 'display_data',
                    'data': {
                        'image/png': img_b64,
                        'text/plain': ['<Figure size ...>']
                    },
                    'metadata': {}
                })

            cell['execution_count'] = exec_counter
            cell['outputs'] = outputs
            exec_counter += 1

    with open(nb_path, 'w', encoding='utf-8') as f:
        json.dump(nb, f, indent=1)
    print(f"Successfully executed and populated: {nb_path} ({exec_counter - 1} code cells)")

def main():
    notebooks = [
        'notebooks/01_EDA.ipynb',
        'notebooks/02_Jump_Detection.ipynb',
        'notebooks/03_Filter_and_PSD.ipynb',
        'notebooks/04_Downsampling.ipynb',
        'notebooks/05_Throttle_to_RPM_and_Vibration_Dynamics.ipynb',
        'notebooks/Filter_and_PSD_extended.ipynb',
        'notebooks/EDA_and_Downsampling_extended.ipynb'
    ]
    for nb in notebooks:
        if os.path.exists(nb):
            execute_notebook(nb)

if __name__ == '__main__':
    main()

