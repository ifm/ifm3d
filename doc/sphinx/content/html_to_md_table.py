import pandas as pd

def html_to_markdown_table(html_string):
    # Read HTML tables into a list of DataFrame objects.
    tables = pd.read_html(html_string)

    # Convert the first table to Markdown.
    markdown_string = tables[0].to_markdown(index=False)

    return markdown_string

# Example usage:
html_string = """
<table>
  <tr>
    <th>Dependency</th>
    <th>Dependent ifm3d module</th>
    <th>Notes</th>
  </tr>
  <tr>
    <td><a href="http://www.cmake.org">CMake</a></td>
    <td>device, framegrabber, swupdater, pcicclient, tools, pybind11</td>
    <td>Meta-build framework</td>
  </tr>
  <tr>
    <td><a href="https://curl.haxx.se/libcurl">Curl</a></td>
    <td>device, tools, swupdater</td>
    <td>Used to help enable command-line based firmware flashing.</td>
  </tr>
  <tr>
    <td><a href="https://github.com/google/glog">Glog</a></td>
    <td>device, framegrabber, swupdater, pcicclient, tools, pybind11</td>
    <td>Logging framework</td>
  </tr>
  <tr>
    <td><a href="https://github.com/google/googletest">Gtest</a></td>
    <td>device, framegrabber, swupdater, pcicclient, tools, pybind11</td>
    <td>Unit testing framework</td>
  </tr>
  <tr>
    <td><a href="http://xmlrpc-c.sourceforge.net/">libxmlrpc</a></td>
    <td>device, pybind11</td>
    <td>XMLRPC client used call into the camera configuration interface</td>
  </tr>
  <tr>
    <td><a href="https://github.com/pybind/pybind11">pybind11</a></td>
    <td>pybind11</td>
    <td>A header-only library that exposes C++ types in Python and vice versa,
    mainly to create Python bindings of existing C++ code.</td>
  </tr>
</table>
"""

import re

def remove_trailing_spaces_in_table_cells(markdown_string):
    # Regular expression pattern to match trailing spaces in table cells.
    pattern = r'[ \t]+(?=\|)'

    # Replace trailing spaces in table cells with nothing.
    cleaned_string = re.sub(pattern, '', markdown_string, flags=re.MULTILINE)

    return cleaned_string

# Example usage:
markdown_string = html_to_markdown_table(html_string)

print(remove_trailing_spaces_in_table_cells(markdown_string))