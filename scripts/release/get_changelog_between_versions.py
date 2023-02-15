from datetime import date
from argparse import ArgumentParser
import re
def get_changelog_between_version():
  filename = "ChangeLog.md"
  filename_out = "CHANGELOG"
  # Open the file for reading
  with open(filename, "r") as file:
    # Read the file contents into a variable
    file_contents = file.read()
    result = re.search(r'(## \d+\.\d+\.\d+ \- \d{4}\-\d{2}-\d{2}\n){1}((.|\n)*?)(## \d+\.\d+\.\d+ \- \d{4}\-\d{2}-\d{2})', file_contents)
    print(result.group(2))
    # Open the file for writing
    with open(filename_out, "w") as file:
        # Write the updated contents to the file
        file.write(result.group(2))


if __name__ == "__main__":
    get_changelog_between_version()
    
   
