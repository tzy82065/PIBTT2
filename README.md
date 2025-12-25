# PIBTT2
Code and Supplementary files for "Scalable Iterative Multi-Agent Path Finding under Turn Constraints"

This respository consists of a code pack including supplementary files `PIBTT2/supplementary_file_PIBTT2` for paper "Scalable Iterative Multi-Agent Path Finding under Turn Constraints", which is submitted to IEEE Robotics and Automation Letters.

## Author, Copyright and Acknowledgement
The code is based on Okumura's implementation of PIBT (https://github.com/Kei18/pibt2/). This is the original PIBT, which exhibits significant high-speed and scalability in classic MAPF and MAPD problems.

Please cite the following paper if you use the code in your published research:

```
@article{okumura2022priority,
  title = {Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding},
  journal = {Artificial Intelligence},
  pages = {103752},
  year = {2022},
  issn = {0004-3702},
  doi = {https://doi.org/10.1016/j.artint.2022.103752},
  author = {Keisuke Okumura and Manao Machida and Xavier Défago and Yasumasa Tamura},
}

# IJCAI-19

@inproceedings{okumura2019priority,
  title={Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding},
  author={Okumura, Keisuke and Machida, Manao and D{\'e}fago, Xavier and Tamura, Yasumasa},
  booktitle={Proceedings of the Twenty-Eighth International Joint Conference on Artificial Intelligence, {IJCAI-19}},
  publisher={International Joint Conferences on Artificial Intelligence Organization},
  pages={535--542},
  year={2019},
  month={7},
  doi={10.24963/ijcai.2019/76},
  url={https://doi.org/10.24963/ijcai.2019/76}
}

@inproceedings{tao2025fast,
  title={Fast Multi-Agent Path Planning with Turn Actions: A Priority Inheritance Approach},
  author={Tao, Zheyu and Yu, Chunlong},
  booktitle={2025 IEEE 21st International Conference on Automation Science and Engineering (CASE)},
  pages={2024--2029},
  year={2025},
  organization={IEEE}
}
```
...

## Description
- Our algorithm adapts PIBT to MAPF with turn actions.
- It is implemented in C++ with CMake(≥v3.16).
- The following files in pibt2 (https://github.com/Kei18/pibt2/) are modified:
orientation.cpp/.hpp, pibt.cpp/.hpp, plan.cpp/.hpp, problem.cpp/.hpp, solver.cpp/.hpp

## Building
```sh
git clone --recursive https://github.com/tzy82065/PIBTT2.git
cd PIBTT2
mkdir build && cd build
cmake ..
make
```

## Usage
```sh
./mapf -i ../instances/mapf/sample.txt -s PIBT -o result.txt -v
```

## Experiment
- The experiment is conducted on 5 MAPF Benchmark maps: empty-32-32, random-32-32-20, room-64-64-8, warehouse-10-20-10-2-2, den520d. Map files `.map` and corresponding scenario files `.scen` can be download on https://movingai.com/benchmarks/mapf/index.html.
- All agents start planning with an orientation "North".

- All instances used in numerical experiments are in `https://github.com/tzy82065/PIBTT2/tree/main/instances/tests`
- Usage: ../run_experiments.sh <map_name> <number_of_agents1> [<number_of_agents2> ...]
- Example: ../run_experiments.sh random-32-32-20 10 20 30
## Visualize
A lot of Thanks to Okumura again! The visualizer module from him can be download in (https://github.com/kei18/mapf-visualizer).
