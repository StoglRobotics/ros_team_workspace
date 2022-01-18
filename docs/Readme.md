# Ros_Team_workspace Documentation
### Installation and building
If you want to edit the docs you first have to install sphinx. From within the docs directory run:
```bash
python -m pip install -r requirements.txt
```
After the requirements have been installed, the docs can then be build if you run:
```bash
make html
```
from within the `docs/` folder.

You can then view the docs by opening the `docs/_build/html/index.html` in your browser.
