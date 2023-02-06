# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))

from docutils import nodes
from sphinx.transforms import SphinxTransform
import version

# -- Project information -----------------------------------------------------

project = 'ifm3d'
copyright = '2021, ifm electronic'
author = 'ifm electronic'
release = version.__version__

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.napoleon',
    'myst_parser',
    'sphinx_tabs.tabs',
]

autosummary_generate = True

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

master_doc = 'index'

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'canonical_url': '',
    'analytics_id': '',
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,

    'logo_only': False,

    # Toc options
    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': -1,
    'includehidden': True,
    'titles_only': False
}
# html_logo = ''
# github_url = ''
# html_baseurl = ''

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']
html_css_files = [
    'custom.css',
]


myst_enable_extensions = [
    "colon_fence",
    "substitution", # This enable the definition of substitution variables (see below)
]

sphinx_tabs_disable_tab_closing = True

myst_url_schemes = ("http", "https", "mailto", "ftp", "relurl")

class RelUrlTransform(SphinxTransform):
    default_priority = 1
    def apply(self, **kwargs) -> None:
        for node in self.document.traverse(nodes.reference):
            if "refuri" in node and node["refuri"].startswith("relurl:"):
                node["refuri"] = node["refuri"][7:]


def filter_bases(app, name, obj, options, bases):
    bases[:] = [None.__class__ if x.__name__ ==
                "pybind11_object" else x for x in bases]


def setup(app):
    app.add_transform(RelUrlTransform)
    app.connect('autodoc-process-bases', filter_bases)

# -------------------------------------------------
# -- Substitution variables
# -------------------------------------------------
myst_substitutions = {
    "ifm3d_gh_url" : "https://github.com/ifm/ifm3d",
    "ifm3d_main_branch":  "main", # The most up to date branch on ifm3d
    "ifm3d_latest_tag_url": "https://github.com/ifm/ifm3d/tags",
    "ifm3d_containers_list_url": "https://github.com/ifm/ifm3d/pkgs/container/ifm3d",
}
