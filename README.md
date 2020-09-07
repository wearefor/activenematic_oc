# activenematic_oc

Optimal Control of Active Nematics using COMSOL and MATLAB

These scripts perform the optimal control calculation presented in https://arxiv.org/abs/2007.14837 that steer a continuum fluid model of an active suspension towards a desired dynamical state. The code utilizes the numerical technique “direct adjoint looping” to iteratively converge on a spatio-temporal field for either activity strength α or applied vorticity g, through consecutive finite element simulations consisting of forward, backward (adjoint), and update (gradient descent) steps.

Software Requirements:
MATLAB ® , written with 2019a, compatibility with other versions is untested
• COMSOL Multiphysics ® with MATLAB LiveLink module, code written for v5.2, compatibility
with other versions is untested
• matlab-ascii-plot, free download https://github.com/kyak/matlab-ascii-plot/releases/tag/0.1.1

Installation:
1. Download or pull
2. Configure AN_adjointloop_setup.m and AN_adjointloop.m
3. Further details in manual.pdf
