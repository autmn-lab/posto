# `Posto` 
## About The Tool

With the increasing autonomous capabilities of cyber-physical systems, the complexity of their models also increases significantly, thus continually posing challenges to existing formal methods for safety verification. In contrast to model checking, monitoring emerges as an effective lightweight, yet practical verification technique capable of delivering results of practical importance with better scalability. Monitoring involves analyzing logs from an actual system to determine whether a specification (such as a safety property) is violated.  Although current monitoring techniques work well in some areas, it has largely been unable to cope with the growing complexity of the models. Monitoring techniques, such as those using reachability methods, may fail to produce results when dealing with complex models like Deep Neural Networks (DNNs). We propose here a novel statistical approach for monitoring that is able to generate results with high probabilistic guarantees. 

`Posto` is a Python-based prototype tool that implements the proposed statistical monitoring technique, enabling an effective monitoring of complex systems, including non-linear systems with DNN-based components, while providing results with high probabilistic guarantees.

![Overview](https://github.com/bineet-coderep/monitor-bb/blob/main/figs/Overview.png)


## Installation

### Dependencies

- [`Python 3.9.x`](https://www.python.org/)
- [`NumPy`](https://numpy.org/)
- [`SciPy`](https://scipy.org/)
- [`mpmath`](https://mpmath.org/)
- [`mpl_toolkits`](https://matplotlib.org/2.2.2/mpl_toolkits/index.html)
- [`tqdm`](https://pypi.org/project/tqdm/2.2.3/)

### Downloading the tool

1. Download the repository to your desired location `/my/location/`:

2. Once the repository is downloaded, please open `~/.bashrc`, and add the line `export MNTR_BB_ROOT_DIR=/my/location/monitor-bb/`, mentioned in the following steps:

   1. ```shell
      vi ~/.baschrc
      ```

   2. Once `.bashrc` is opened, please add the location, where the tool was downloaded, to a path variable `MNTR_BB_ROOT_DIR` (This step is crucial to run the tool):

      1. ```shell
         export MNTR_BB_ROOT_DIR=/my/location/monitor-bb/
         ```

## Artifact Evaluation

This section outlines the steps to recreate the plots presented in the paper. It is worth noting that the safety verification method proposed in our work is inherently statistical. Consequently, the reproduction of the plots will not yield an exact match but rather a stochastic recreation. In other words, the plots are recreated using the same set of parameters as in the draft, which define the distributions. The recreated plots represent one possible outcome from that distribution, hence the term ‘stochastic recreation.’ As a result, some figures that were initially inferred to be safe in the draft may yield unsafe results (or vice versa) during this process.

It is important to note that this does not affect the soundness of our method. Rather, it reaffirms the stochastic nature of our work, and such stochastic recreations demonstrate the influence of various parameters on the proposed method, as explored in the research questions of the paper. In simpler terms, while the recreated plots may not be identical (in a deterministic sense), they are stochastically equivalent.

To elaborate on the sources of stochasticity, the following factors contribute to the variability in the generated plots (testing this by running the scripts multiple times could be a good approach):

1. Although the same parameters (logging probability and log noise) are used to generate the logs, the logs themselves are random, though stochastically equivalent, as they are derived from the same distribution.
2. Additionally, the safety verification process is inherently stochastic, meaning that safety inferences could vary across different iterations. 

### Recreating Figs. 3(a)-(c)

1. **To recreate fig. 3(a) (or fig. 3(c)), perform the following steps:**

   1. Make sure you are in location `/my/location/monitor-bb/`

      * ```bash
        cd /my/location/monitor-bb/
        ```

   2. Set the parameter `FIG='3(a)'` (or `FIG='3(c)'`) in line 32 of the file [`/my/location/monitor-bb/Parameters.py`](https://github.com/bineet-coderep/monitor-bb/blob/main/Parameters.py).

      * ```python
        FIG='3(a)' # Choose from {3(a), 3(b), 3(c), 4(a), 4(b), 4(c), 4(d), 5(a), 5(b), 5(c), 5(d)}
        ```

   3. Run the python script [`Jet2.py`](https://github.com/bineet-coderep/monitor-bb/blob/main/src_artifact/Jet2.py) from directory `/my/location/monitor-bb/`

      * ```bash
        python src_artifact/Jet2.py
        ```

      * **NOTE:** This will generate fig. 3(a) first, followed by fig. 3(c). Be sure to close the figure window before generating the next figure.

      * ![3(a)](https://github.com/bineet-coderep/monitor-bb/blob/main/figs/3(a).png)

      * ![3(c)](https://github.com/bineet-coderep/monitor-bb/blob/main/figs/3(c).png)

This experiment should take a few seconds.

2. **To recreate fig. 3(b), perform the following steps:**

   1. Make sure you are in location `/my/location/monitor-bb/`

      * ```bash
        cd /my/location/monitor-bb/
        ```

   2. Set the parameter `FIG='3(b)'` in line 32 of the file [`/my/location/monitor-bb/Parameters.py`](https://github.com/bineet-coderep/monitor-bb/blob/main/Parameters.py).

      * ```python
        FIG='3(b)' # Choose from {3(a), 3(b), 3(c), 4(a), 4(b), 4(c), 4(d), 5(a), 5(b), 5(c), 5(d)}
        ```

   3. Run the python script [`fig3b.py`](https://github.com/bineet-coderep/monitor-bb/blob/main/src_artifact/fig3b.py) from directory `/my/location/monitor-bb/`

      * ```bash
        python src_artifact/fig3b.py
        ```

      * ![3(b)](https://github.com/bineet-coderep/monitor-bb/blob/main/figs/3(b).png)

This experiment should take a fraction of seconds.

### Recreating Figs. 4(a)-(c)

**Following steps are to specifically recreate fig. 4(a), but the same steps could be used to generate figs. 4(b)-(c), as long as the parameter value is set accordingly in step 2:**

1. Make sure you are in location `/my/location/monitor-bb/`

   * ```bash
     cd /my/location/monitor-bb/
     ```

2. Set the parameter `FIG='4(a)'` (or `FIG='4(b)'` and others) in line 32 of the file [`/my/location/monitor-bb/Parameters.py`](https://github.com/bineet-coderep/monitor-bb/blob/main/Parameters.py).

   * ```python
     FIG='4(a)' # Choose from {3(a), 3(b), 3(c), 4(a), 4(b), 4(c), 4(d), 5(a), 5(b), 5(c), 5(d)}
     ```

3. Run the python script [`Jet.py`](https://github.com/bineet-coderep/monitor-bb/blob/main/src_artifact/Jet.py) from directory `/my/location/monitor-bb/`

   * ```bash
     python src_artifact/Jet.py
     ```

   * ![Fig4](https://github.com/bineet-coderep/monitor-bb/blob/main/figs/Fig4.png)
   

This experiment may take about 20 seconds.

### Recreating Figs. 5(a)-(c)

**Following steps are to specifically recreate fig. 5(a), but the same steps could be used to generate figs. 5(b)-(c), as long as the parameter value is set accordingly in step 2:**

1. Make sure you are in location `/my/location/monitor-bb/`

   * ```bash
     cd /my/location/monitor-bb/
     ```

2. Set the parameter `FIG='5(a)'` (or `FIG='5(b)'` and others) in line 32 of the file [`/my/location/monitor-bb/Parameters.py`](https://github.com/bineet-coderep/monitor-bb/blob/main/Parameters.py).

   * ```python
     FIG='5(a)' # Choose from {3(a), 3(b), 3(c), 4(a), 4(b), 4(c), 4(d), 5(a), 5(b), 5(c), 5(d)}
     ```

3. Run the python script [`VanderPol.py`](https://github.com/bineet-coderep/monitor-bb/blob/main/src_artifact/VanderPol.py) from directory `/my/location/monitor-bb/`

   * ```bash
     python src_artifact/VanderPol.py
     ```

   * ![Fig4](https://github.com/bineet-coderep/monitor-bb/blob/main/figs/Fig5.png)

   
