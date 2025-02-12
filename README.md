# MBB
Monitoring of black box systems


## Installation

### Dependencies

- [`Python 3.9.x`](https://www.python.org/)
- [`NumPy`](https://numpy.org/)
- [`SciPy`](https://scipy.org/)
- [`mpmath`](https://mpmath.org/)
- [`mpl_toolkits`](https://matplotlib.org/2.2.2/mpl_toolkits/index.html)

### Downloading the tool

1. Download the repository to your desired location `/my/location/`:

2. Once the repository is downloaded, please open `~/.bashrc`, and add the line `export MNTR_BB_ROOT_DIR=/my/location/monitor-bb/`, mentioned in the following steps:

   1. ```shell
      vi ~/.baschrc
      ```

   2. Once `.bashrc` is opened, please add the location, where the tool was downloaded, to a path variable `ULS_ROOT_DIR` (This step is crucial to run the tool):

      1. ```shell
         export MNTR_ROOT_DIR=/my/location/monitor-bb/
         ```

## Running the Tool

We currently offer two case studies:

1. Jet Model (Fig. 4)
2. Van der Pol Oscillator (Fig. 6)

Here, we illustrate the Jet Model case study, as the other one can be run in similar fashion.

1. ```shell
   cd src_artifact/
   ```

2. ```shell
   python Jet.py
   ```

   1. This will sequentially generated Figs. (a)-(d)

