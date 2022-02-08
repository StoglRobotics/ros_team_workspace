# Ros_Team_workspace Documentation

We use [sphinx](https://www.sphinx-doc.org/en/master/) for our docs. For the building of the multiversion docs  [sphinx-multiversion](https://holzhaus.github.io/sphinx-multiversion/master/index.html#) is used. The docs itself are written in [restructuredtext format](https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html) (*.rst).

## Installation and building
### Installation
If you want to edit the docs you first have to install all the required dependencies. From within the docs directory run:
```bash
python -m pip install -r requirements.txt
```
### Building
After the requirements have been installed, the docs can then be build if you run:
```bash
make html
```
from within the `docs/` folder. You can then view the docs by opening the `docs/_build/html/index.html` in your browser. Changes you made should immediately be visible.

### Building multiversion
If you want to build multiversion docs you have to run:
```bash
make multiversion
```
from within the `docs/` folder. You can then view the docs by opening the `docs/_build/html/index.html`.
:exclamation: However, be aware that changes to the multiversion docs only appear after committing them. :exclamation: If you want to see them immediate, you have to build using `make html`.
