# Minimal makefile for Sphinx documentation
#
# $(0) is meant as shortcut for $(SPHINXOPTS) to pass them on cmd line
#

# You can set these variables from the command line.
SPHINXBUILD   = python3 -m sphinx
SOURCEDIR     = .
BUILDDIR      = _build
SPHINXOPTS    = -c .

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

multiversion: Makefile
	sphinx-multiversion $(SPHINXOPTS) "$(SOURCEDIR)" "$(BUILDDIR)/html"
	@echo "<html><head><meta http-equiv=\"refresh\" content=\"0; url=foxy/index.html\" /></head></html>" > "$(BUILDDIR)"/html/index.html

# https://www.gnu.org/software/make/manual/html_node/Phony-Targets.html
.PHONY: help Makefile multiversion

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
