
## Code Formatting

Formatters and linters are integrated to the repository as pre-commit-hooks and uses `pre-commit` tool to execute them. For list of hooks and their sources check `.pre-commit-config.yaml` file.

To install `pre-commit` tool locally use:
```
pip3 install pre-commit  # (prepend `sudo` if you want to install it system wide)
```

Then set it up for automatic execution on every commit using:
```
pre-commit install
```

To run it initially over the whole repo you can use:
```
pre-commit run -a
```

**NOTE**: make sure that you have sources your ROS workspace before running `pre-commit` because it uses some office ROS 2 `ament_*` formatters.

Sometimes there might a need to avoid formatting of a certain commit. If so add `-n` flag to your `git commit` command to skip the `pre-commit` checks.
