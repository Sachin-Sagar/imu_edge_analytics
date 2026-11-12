#!/usr/bin/env python3
"""
HTML Report Generator with MathJax and GitHub Markdown styling.
Renders markdown files (such as docs/final_report.md, docs/phase4_report.md, etc.)
into standalone HTML documents with LaTeX math rendering and responsive styling.
"""
import os
import sys
try:
    import markdown
except ImportError:
    sys.path.append('/usr/lib/python3/dist-packages')
    import markdown

def render_md_to_html(md_path, html_path, title="Motocross Jump Analytics Report"):
    """Render a markdown file to a standalone styled HTML file."""
    if not os.path.exists(md_path):
        raise FileNotFoundError(f"Markdown file not found: {md_path}")

    with open(md_path, 'r', encoding='utf-8') as f:
        md_content = f.read()

    # Convert markdown to html with tables and fenced code
    html_body = markdown.markdown(md_content, extensions=['tables', 'fenced_code', 'nl2br'])

    html_template = f'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>{title}</title>
<style>
body {{
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    line-height: 1.6;
    color: #24292e;
    max-width: 1100px;
    margin: 0 auto;
    padding: 40px 20px;
    background: #fdfdfd;
}}
h1, h2, h3, h4 {{
    color: #1a1a1a;
    margin-top: 1.5em;
    margin-bottom: 0.5em;
    border-bottom: 1px solid #eaecef;
    padding-bottom: 0.3em;
}}
h1 {{
    border-bottom: 2px solid #2b5c8f;
    color: #2b5c8f;
    font-size: 2.2em;
}}
table {{
    border-collapse: collapse;
    width: 100%;
    margin: 20px 0;
    font-size: 0.95em;
    box-shadow: 0 1px 3px rgba(0,0,0,0.1);
    background: #fff;
}}
th, td {{
    border: 1px solid #dfe2e5;
    padding: 10px 14px;
    text-align: left;
}}
th {{
    background-color: #f6f8fa;
    font-weight: 600;
}}
tr:nth-child(even) {{
    background-color: #fcfcfc;
}}
code {{
    background: #f0f3f6;
    padding: 2px 6px;
    border-radius: 4px;
    font-family: SFMono-Regular, Consolas, "Liberation Mono", Menlo, monospace;
    font-size: 0.9em;
}}
pre code {{
    display: block;
    padding: 12px;
    overflow-x: auto;
    background: #282c34;
    color: #abb2bf;
    border-radius: 6px;
}}
img {{
    max-width: 100%;
    height: auto;
    border: 1px solid #e1e4e8;
    border-radius: 6px;
    margin: 15px 0;
    box-shadow: 0 2px 6px rgba(0,0,0,0.08);
}}
blockquote {{
    border-left: 4px solid #2b5c8f;
    padding: 10px 20px;
    margin: 20px 0;
    color: #555;
    background: #f4f8fb;
    border-radius: 0 6px 6px 0;
}}
.metric-card {{
    background: #fff;
    border: 1px solid #e1e4e8;
    border-radius: 8px;
    padding: 18px;
    margin: 15px 0;
    box-shadow: 0 2px 4px rgba(0,0,0,0.05);
}}
</style>
<script>
MathJax = {{
  tex: {{
    inlineMath: [['$', '$'], ['\\\\(', '\\\\)']],
    displayMath: [['$$', '$$'], ['\\\\[', '\\\\]']],
    processEscapes: true
  }}
}};
</script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>
</head>
<body>
{html_body}
</body>
</html>'''

    os.makedirs(os.path.dirname(os.path.abspath(html_path)), exist_ok=True)
    with open(html_path, 'w', encoding='utf-8') as f:
        f.write(html_template)
    print(f"Generated HTML report: {html_path}")

def main():
    # Render phase3 report
    if os.path.exists('docs/phase3_report.md'):
        render_md_to_html('docs/phase3_report.md', 'docs/phase3_report.html',
                          title="Phase 3 Technical Report: Objective 2 (Filter Design & PSD)")

    # Render phase4 report
    if os.path.exists('docs/phase4_report.md'):
        render_md_to_html('docs/phase4_report.md', 'docs/phase4_report.html',
                          title="Phase 4 Technical Report: Objective 3 (Downsampling & Power Efficiency)")
        # Also mirror to results/phase4_downsampling/report.html
        render_md_to_html('docs/phase4_report.md', 'results/phase4_downsampling/report.html',
                          title="Phase 4 Technical Report: Objective 3 (Downsampling & Power Efficiency)")

    # Render final report if exists
    if os.path.exists('docs/final_report.md'):
        render_md_to_html('docs/final_report.md', 'docs/final_report.html',
                          title="Motocross Jump Analytics — Technical Report (Motion Analytics Report)")

if __name__ == '__main__':
    main()
