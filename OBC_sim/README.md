# MSP Simulator

A configurable simulator for simulating OBC side of MSP communication.
Only available for Arduino Due at the moment.

## Get started
MSP is used as a git submodule, so start by initializing it:
```bash
git submodule update --init
```

Edit `config.txt` to change the configuration of the simulator. Instructions
for how to configure are present in the file.

After editing `config.txt`, run `setup.py` with Python 3 (it will not work with
Python 2). This will generate the folder `target/obcsim/` that contains the
Arduino Due code generated according to `config.txt`.

If you need to quickly switch between multiple configurations, e.g.
`config-stresstest.txt` `config-simple.txt`, you can type
`python3 setup.py [filename]` to use a different configuration file than
`config.txt`. If no argument is given, `setup.py` will default to
`config.txt`.
