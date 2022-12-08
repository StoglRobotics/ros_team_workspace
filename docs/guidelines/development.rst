============================
Recommendation for developments
============================

This documents proposes guidelines for developers, which increases their work efficiency and synergy within the development team. 
**NOTE**: All the proposal here are the resuls of authors' personal experiences. Saying that, if you don't like some of it, you are free to change what you want and need (and hopefully propose it as PR).


How to use pre-commit :
""""""""""""""""""""""""""""""""""""

1. Install pre-commit using the following command;

   pip install pre-commit

2. from the root directory of the repository run the following command;
   
   pre-commit install

3. add a pre-commit configuration file named .pre-commit-config.yaml, if not already existing in the root directory of the repository. If collaborating in a team contact your team head to access the .pre-commit-config.yaml file that is common for ally our team members;


4.  to update the configuration file run the following command;

   pre-commit autoupdate

5. manually run following command of pre-commit to check status of all hooks;

   pre-commit run -a

6. for your information, the following command is automatically run after every commit you make;

   pre-commit install

Hence expect the status of hooks on the terminal after every commit


