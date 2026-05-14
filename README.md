# ROS 2 Racing Playground


This repository reuses parts of the ForzaETH Race Stack for training, experimentation, and development.
It is not the official complete race stack; it is a testing playground built on top of the
[D-ITET Center for Project Based Learning (PBL)](https://pbl.ee.ethz.ch/) work at ETH Zurich.

<a href="https://arxiv.org/abs/2403.11784">
    <img src="https://img.shields.io/badge/arXiv.org-2403.11784-b31b1b" alt="arXiv e-print Badge">
</a>

## Getting started

After installation, the car (or the simulation environment) is ready to be tested. For examples on how to run the different modules on the car, refer to the [`stack_master` README](./stack_master/README.md).

## Acknowledgement
This project would not be possible without the use of multiple great open-sourced code bases as listed below:

- [f1tenth_system](https://github.com/f1tenth/f1tenth_system)
- [F1TENTH Racecar Simulator](https://github.com/f1tenth/f1tenth_simulator)
- [Veddar VESC Interface](https://github.com/f1tenth/vesc)
- [Cartographer](https://github.com/cartographer-project/cartographer)
- [Cartographer ROS Integration](https://github.com/cartographer-project/cartographer_ros)
- [global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization)
- [RangeLibc](https://github.com/kctess5/range_libc)
- [BayesOpt4ROS](https://github.com/IntelligentControlSystems/bayesopt4ros)
- [cpu_monitor](https://github.com/alspitz/cpu_monitor)

### Problems
If you are having problem with the SIM (no car/scans showing for example), try the setup once again:
```bash
source ~/ws/src/race_stack/.install_utils/f110_sim_setup.sh
```

If your joystick is not working, try the following while the controller is connected:
```bash
sudo chmod 666 /dev/input/js0
sudo chmod 666 /dev/input/event*
```

## Citing ForzaETH Race Stack

If you found our race stack helpful in your research, we would appreciate if you cite it as follows:
```
@article{baumann2024forzaeth,
  title={ForzaETH Race Stack—Scaled Autonomous Head-to-Head Racing on Fully Commercial Off-the-Shelf Hardware},
  author={Baumann, Nicolas and Ghignone, Edoardo and K{\"u}hne, Jonas and Bastuck, Niklas and Becker, Jonathan and Imholz, Nadine and Kr{\"a}nzlin, Tobias and Lim, Tian Yi and L{\"o}tscher, Michael and Schwarzenbach, Luca and others},
  journal={Journal of Field Robotics},
  year={2024},
  publisher={Wiley Online Library}
}
```
