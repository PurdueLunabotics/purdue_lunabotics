# Configuration file for the Sphinx documentation builder.
#

import os
import sys

sys.path.insert(0, os.path.abspath('../lunabot_control/scripts'))

# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Purdue Lunabotics Software Documentation'
copyright = '2023, Purdue Lunabotics'
author = 'Purdue Lunabotics'
release = '0.1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'sphinx_rtd_theme',
    ]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']


#import sys
#from sphinx.highlighting import lexers
#from pygments.lexers.c_cpp import CppLexer

#sys.path.insert(0, '..')

#lexers['ino'] = CppLexer()