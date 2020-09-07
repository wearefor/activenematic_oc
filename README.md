# activenematic_oc

Optimal Control of Active Nematics using COMSOL and MATLAB

These scripts perform the optimal control calculation presented in https://arxiv.org/abs/2007.14837 that steer a continuum fluid model of an active suspension towards a desired dynamical state. The code utilizes the numerical technique “direct adjoint looping” to iteratively converge on a spatio-temporal field for either activity strength α or applied vorticity g, through consecutive finite element simulations consisting of forward, backward (adjoint), and update (gradient descent) steps.

Software Requirements:
1. MATLAB ® , written with 2019a, compatibility with other versions is untested
2. COMSOL Multiphysics ® with MATLAB LiveLink module, code written for v5.2, compatibility
with other versions is untested
3. matlab-ascii-plot, free download https://github.com/kyak/matlab-ascii-plot/releases/tag/0.1.1

Installation:
1. Download or pull
2. Configure AN_adjointloop_setup.m and AN_adjointloop.m
3. Further details in manual.pdf

Acknowledgements:
This research was conducted at Bradeis University, Waltham, Massachusetts USA and University of Nebraska, Lincoln, Nebraska, USA
This work was supported by the NSF MRSEC-1420382, NSF-DMR-2011486, NSFDMR1810077, and DMR-1855914 and the University of Nebraska startup fund.
Computational resources were provided by the NSF through XSEDE computing resources (MCB090163) and the Brandeis HPCC, which is partially supported by NSFDMR-2011486. 
