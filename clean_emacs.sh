#!/bin/sh
#
# Recursively removes emacs backup files from your source directory.
#
echo "Cleaning up emacs backup files..."
find . -name "*~" | xargs rm -f
echo "OK"