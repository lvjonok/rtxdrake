# Setup notes

This document describes the steps one should take to set up the `rtxdrake` tool. I will use conda environment to install the necessary packages and run the tool.

1. Initialize the conda environment (mamba is suggested):

   ```bash
   mamba create -n rtxdrake python=3.10
   mamba activate rtxdrake
   ```

2. Install OpenUSD:

   - I have found that `drake` sends `.gltf` file format to meshcat and I had to find a tool that can convert it to `.usda` format.
   - [guc](https://github.com/pablode/guc) absolutely nails it.
   - To install `guc` you first have to build [OpenUSD](https://github.com/PixarAnimationStudios/OpenUSD) from source.

   ```bash
   mamba install xorg-libxt
   git clone https://github.com/PixarAnimationStudios/OpenUSD.git
   python OpenUSD/build_scripts/build_usd.py /home/lvjonok/miniforge3/envs/rtxdrake --build-python-info $CONDA_PREFIX/bin/python3.10 $CONDA_PREFIX/include/python3.10 $CONDA_PREFIX/lib/python3.10 3.10 --no-python
   ```

   - Then we install `guc`:

   ```bash
    git clone https://github.com/pablode/guc --recursive
    mkdir guc/build && cd guc/build
    cmake .. -Wno-dev -DCMAKE_BUILD_TYPE=Release
    cmake --build . -j8 --config Release
   ```

3. Install `isaacsim`:

   - There are different ways, but I have found that installing it from pip is easier and gives clearer view on which python interpreter is used afterwards.

   - [Here is a throrough guide](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html)

4. Install `rtxdrake`:

   - Clone the repository:

   ```bash
   git clone https://github.com/lvjonok/rtxdrake.git
   cd rtxdrake
   pip install -e .
   ```

5. Try to run everything:

   - First, start the main example script for isaacsim:

   ```bash
   python main.py
   ```

   - Then, in another terminal, run the Drake simulation:

   ```bash
   python sample.py
   ```

I hope I have covered everything. If you have any questions, feel free to create an issue and we can expand this document together.
