from datetime import date
from argparse import ArgumentParser
import re

def update_file(filename, old_text, new_text):
    # Define the keyword and replacement text
    keyword = old_text
    replacement_text = new_text

    # Open the file for reading
    with open(filename, "r") as file:
        # Read the file contents into a variable
        file_contents = file.read()

    # Replace the keyword with the replacement text
    file_contents = file_contents.replace(keyword, replacement_text, 1)

    # Open the file for writing
    with open(filename, "w") as file:
        # Write the updated contents to the file
        file.write(file_contents)

    print("Keyword replaced successfully")

def update_changelog(version):
    filename = "ChangeLog.md"
    text_to_search = "[Unreleased]"
    text_to_replace = text_to_search +"\n\n## "+ version +" - " + date.today().strftime("%Y-%m-%d")
    update_file(filename,text_to_search,text_to_replace)

def update_version_file(version):
    result = re.search('^v(\d+).(\d+).(\d+)',version)
    major_version = result.group(1)
    minor_version = result.group(2)
    patch_version = result.group(3)
    version_for_file = version +'*'+version+'*'+major_version+'*'+minor_version+'*'+patch_version
    # Open the file for reading
    with open("VERSION", "w") as file:
        # Write the updated contents to the file
        file.write(version_for_file)

if __name__ == "__main__":
    
    parser = ArgumentParser(description='Argument parser for ifm3d release')

    parser.add_argument('--version', default=None, required=True, help='version of ifm3d')
    args = parser.parse_args()
    version = args.version
    match_found = bool(re.match("^[v]{1}\d{1,2}\.\d{1,2}\.\d{1,3}", version))
    if match_found == False:
        raise Exception("Error in version string, please format it to vx.x.x")
    else:
        update_changelog(version[1:])
        update_version_file(version)
