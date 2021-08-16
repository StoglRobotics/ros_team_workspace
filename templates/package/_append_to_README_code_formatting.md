
## Code Formatting

This repository uses `pre-commit` tool for code formatting.
The tool checks formatting each time you commit to a repository.
To install it locally use:
  ```
  pip3 install pre-commit  # (prepend `sudo` if you want to install it system wide)
  ```

To run it initially over the whole repo you can use:
  ```
  pre-commit run -a
  ```

If you get error that something is missing on your computer, do the following for:

  - `clang-format-10`
     ```
     sudo apt install clang-format-10
     ```
