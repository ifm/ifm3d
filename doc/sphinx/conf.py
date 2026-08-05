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

import enum
import re

from docutils import nodes
from sphinx.transforms import SphinxTransform
from sphinx.util import logging as sphinx_logging
import ifm3dpy_version

logger = sphinx_logging.getLogger(__name__)

# -- Project information -----------------------------------------------------

project = "ifm3d"
copyright = "2021, ifm electronic"
author = "ifm electronic"
release = ifm3dpy_version.__version__
version = ifm3dpy_version.__version__

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.intersphinx",
    "sphinx.ext.napoleon",
    "myst_parser",
    "sphinx_tabs.tabs",
]

autosummary_generate = True
autosummary_imported_members = False

# The pybind11 bindings carry hand written numpy style docstrings, so let
# napoleon own the rendering of the ``Parameters``/``Returns`` sections and keep
# autodoc from appending a second, redundant set of types.
napoleon_use_param = True
napoleon_use_rtype = True
napoleon_preprocess_types = True

autodoc_typehints = "signature"
autodoc_member_order = "groupwise"
autodoc_default_options = {
    "show-inheritance": True,
    "undoc-members": True,
}

# Resolve short names such as ``SemVer`` or ``buffer_id`` that appear in the
# docstrings against the module they are documented in.
python_use_unqualified_type_names = True

intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "numpy": ("https://numpy.org/doc/stable", None),
}

# Several ifm3dpy types subclass a foreign base: ``ifm3dpy.Buffer`` extends
# numpy.ndarray and every pybind11 enum extends enum.IntEnum (and therefore
# int). The inherited APIs are documented upstream and only bury the ifm3d
# members, so they are excluded from the generated pages (see the
# ``inherited-members`` option in _templates/class.rst) and must be kept out of
# the summary tables as well.
def _is_enum_alias(member_name, member):
    """Whether ``member_name`` is a second name for an existing enum value.

    Aliases are real entries in the class ``__dict__``, so autodoc documents
    them as separate attributes. That is noise at best, and actively harmful
    for ``ifm3dpy.logging.LogLevel.None``: the Python domain resolves type
    annotations with a fuzzy suffix search, so every ``-> None`` return
    annotation in the whole documentation matched that target instead of the
    builtin.
    """
    return isinstance(member, enum.Enum) and member.name != member_name


def _defined_outside_ifm3dpy(cls, member_name):
    """Whether the class that defines ``member_name`` is not part of ifm3dpy."""
    for base in getattr(cls, "__mro__", ()):
        if member_name in vars(base):
            return not getattr(base, "__module__", "").startswith("ifm3dpy")
    return False


def _documented_members(fullname, names):
    """Filter member names down to the ones that actually get a doc page.

    ``autosummary`` builds its summary tables from ``dir()``, which does not
    know about the members autodoc drops. Without this filter every dropped
    member becomes a link to a page that was never generated.
    """
    import importlib

    module_name, _, class_name = fullname.rpartition(".")
    try:
        obj = getattr(importlib.import_module(module_name), class_name)
    except (ImportError, AttributeError):
        logger.warning(
            "could not import %s, autosummary members of %s are not filtered",
            module_name,
            fullname,
        )
        return list(names)

    visible = []
    for member_name in names:
        if _defined_outside_ifm3dpy(obj, member_name):
            continue
        member = getattr(obj, member_name, None)
        if _is_enum_alias(member_name, member):
            continue
        doc = getattr(member, "__doc__", None)
        if doc and ":meta hidden:" in doc:
            continue
        # pybind11 gives classes that cannot be constructed from Python an
        # ``__init__(*args, **kwargs)`` stub carrying object's stock docstring.
        if member_name == "__init__" and doc == object.__init__.__doc__:
            continue
        visible.append(member_name)
    return visible


autosummary_context = {"documented_members": _documented_members}

# ``nitpicky`` surfaces every unresolved cross reference so broken type links do
# not silently render as plain text.
nitpicky = True
nitpick_ignore = [
    # pybind11 emits these for its internal machinery.
    ("py:class", "pybind11_object"),
    ("py:class", "pybind11_builtins.pybind11_object"),
    # Provided by pybind11's stl/typing casters, not importable objects.
    ("py:class", "numpy.typing.ArrayLike"),
]
nitpick_ignore_regex = [
    ("py:class", r"collections\.abc\..*"),
    ("py:class", r"^(int|float|str|bool|bytes|dict|list|tuple|set|object|None)$"),
    # Parameterized generics such as ``list[int]`` are not resolvable as a
    # single class target by the Python domain, which additionally splits them
    # on commas, so ``dict[str, str]`` also yields the fragment ``str]``.
    ("py:class", r"^(list|dict|tuple|set|frozenset|Optional|Union)\[.*"),
    ("py:class", r"^Callable\[\[.*"),
    ("py:class", r"^(int|float|str|bool|bytes|object|None|Any)\]$"),
    # numpy scalar type aliases used in the buffer documentation.
    ("py:class", r"^numpy\.(u?int\d+|float\d+)$"),
]


# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

master_doc = "index"

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"
html_theme_options = {
    "canonical_url": "",
    "analytics_id": "",
    "display_version": True,
    "prev_next_buttons_location": "bottom",
    "style_external_links": False,
    "logo_only": False,
    # Toc options
    "collapse_navigation": True,
    "sticky_navigation": True,
    "navigation_depth": -1,
    "includehidden": True,
    "titles_only": False,
}
# html_logo = ''
# github_url = ''
# html_baseurl = ''

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]
html_css_files = [
    "custom.css",
]


myst_enable_extensions = [
    "colon_fence",
    "substitution",  # This enable the definition of substitution variables (see below)
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
    # ``pybind11_object`` is an implementation detail with no documentation to
    # link to. Report the class as deriving from ``object`` instead; mapping it
    # to ``NoneType`` renders as "Bases: None", which the Python domain then
    # happily resolves to the unrelated ``ifm3dpy.logging.LogLevel.None``.
    bases[:] = [object if x.__name__ == "pybind11_object" else x for x in bases]


def skip_member(app, what, name, obj, skip, opts):
    # ``__doc__`` is ``None`` for a number of pybind11 types and for the module
    # dunders autosummary inspects, so guard before searching it. Returning
    # ``None`` leaves the decision to the default autodoc behaviour.
    if _is_enum_alias(name, obj):
        return True
    doc = getattr(obj, "__doc__", None)
    if doc and ":meta hidden:" in doc:
        return True
    return None


_OVERLOAD_HEADER = "Overloaded function."


# pybind11 3.x names its numeric caster ``typing.SupportsInt`` /
# ``typing.SupportsFloat`` in argument position, because the caster also accepts
# anything implementing ``__index__`` or ``__int__``. That is accurate, but it
# renders as a link into the ``typing`` module where a reader only wants to know
# that they should pass a number.
#
# The rewrite is documentation only. The runtime keeps accepting the wider set,
# so every value the documentation now describes remains valid, and the
# generated ``.pyi`` stubs keep the precise protocol for type checkers. Do not
# "fix" this by adding ``py::arg().noconvert()``: that does render as ``int``,
# but it also stops the argument accepting numpy integer scalars.
_NUMERIC_PROTOCOLS = {"SupportsInt": "int", "SupportsFloat": "float"}

_NUMERIC_PROTOCOL_RE = re.compile(
    r"\b(?:typing\.)?(" + "|".join(_NUMERIC_PROTOCOLS) + r")\b"
)


def _simplify_numeric_protocols(text):
    if not text:
        return text
    return _NUMERIC_PROTOCOL_RE.sub(
        lambda match: _NUMERIC_PROTOCOLS[match.group(1)], text
    )


# An enum class is never called by a reader: the members are the API. Autodoc
# does not know that and introspects ``EnumMeta.__call__``, which is the
# functional constructor, giving every enum a signature along the lines of
# ``(value, names=None, *, module=None, qualname=None, type=None, start=1)``
# (rendered abbreviated as ``(value[, names, module, qualname, ...])``).
def _is_enum_class(obj):
    return isinstance(obj, type) and issubclass(obj, enum.Enum)


def process_signature(app, what, name, obj, options, signature, return_annotation):
    """Clean up the signatures autodoc derives from pybind11 objects.

    ``autodoc-process-signature`` is emitted with ``emit_firstresult``, so the
    first handler returning a value wins. Everything that rewrites a signature
    has to live here rather than in a second handler.
    """
    if _is_enum_class(obj):
        # ``None`` makes autodoc emit no argument list at all.
        return (None, None)

    return (
        _simplify_numeric_protocols(signature),
        _simplify_numeric_protocols(return_annotation),
    )


def simplify_numeric_protocols_in_docstring(app, what, name, obj, options, lines):
    """Apply the same rewrite to signatures left inline in a docstring."""
    for index, line in enumerate(lines):
        if _NUMERIC_PROTOCOL_RE.search(line):
            lines[index] = _simplify_numeric_protocols(line)


def warn_on_overloads(app, what, name, obj, options, lines):
    """Flag pybind11 overloads, which cannot be documented properly.

    pybind11 renders an overloaded function as an opaque ``(*args, **kwargs)``
    signature plus a single docstring concatenating every overload into a
    numbered list. The numpydoc sections of each entry end up indented inside
    that list, so napoleon does not recognise them and the parameter and return
    types are emitted as plain text instead of cross references.

    Bind a single wrapper dispatching to the C++ overloads instead.
    """
    if _OVERLOAD_HEADER in lines:
        logger.warning(
            "%s is an overloaded pybind11 function, so its parameter and "
            "return types cannot be cross referenced. Bind a single wrapper "
            "dispatching to the C++ overloads instead.",
            name,
            type="autodoc",
        )


def setup(app):
    app.add_transform(RelUrlTransform)
    app.connect("autodoc-process-bases", filter_bases)
    app.connect("autodoc-skip-member", skip_member)
    app.connect("autodoc-process-docstring", warn_on_overloads)
    app.connect("autodoc-process-signature", process_signature)
    app.connect("autodoc-process-docstring", simplify_numeric_protocols_in_docstring)


# -------------------------------------------------
# -- Substitution variables
# -------------------------------------------------
myst_substitutions = {
    "ifm3d_gh_url": "https://github.com/ifm/ifm3d",
    "ifm3d_main_branch": "main",  # The most up to date branch on ifm3d
    "ifm3d_latest_tag_url": "https://github.com/ifm/ifm3d/tags",
    "ifm3d_containers_list_url": "https://github.com/ifm/ifm3d/pkgs/container/ifm3d",
    "ifm3d_version": ifm3dpy_version.__version__,
}
