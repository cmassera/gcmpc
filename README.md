# Guaranteed Cost Approach to Robust Model Predictive Control of Uncertain Linear Systems

This repository implements the Guaranteed Cost Model Predictive Controller (GCMPC) and contains complementary material for the paper "A Guaranteed Cost Approach to Robust Model Predictive Control of Uncertain Linear Systems". 

It contains:
- GCMPC Matlab implementation;
- The source code for the numerical examples of the paper;
- Complimentary material with: 
  - Proof of Lemma 3;
  - Discussion on the independence of controllers in Corollary 1.

## Pre-requisites

The source code in this repository requires:

- [MATLAB](http://www.mathworks.com/products/matlab/)

and

- [MATLAB Symbolic Toolbox](https://www.mathworks.com/products/symbolic.html)
- [YALMIP Toolbox](http://users.isy.liu.se/johanl/yalmip/)
- [Mosek](https://www.mosek.com)

Installed and referenced in MATLAB's path.

Note: Mosek has free-licenses for Academia.

### Installing pre-requisites

 1. Create a folder named **tbxmanager**
 2. Go to this folder in Matlab
 3. Execute `urlwrite('http://www.tbxmanager.com/tbxmanager.m', 'tbxmanager.m');`
 4. Execute `tbxmanager install yalmip`
 5. Edit/create startup.m in your Matlab startup folder and add `tbxmanager restorepath` there
 6. Install Mosek through the steps in their website/installer
 7. Add mosek to Matlab's path

## Running the examples

All examples are self contained and can be directly run. For example:

```matlab
>> gcmpc_l2norm
>> rmpc_enum
>> plot_results
```

## Citing

@article{massera2016guaranteed,
title={A Guaranteed Cost Approach to Robust Model Predictive Control of Uncertain Linear Systems},
author={Massera, Carlos M and Terra, Marco H and Wolf, Denis F},
journal={arXiv preprint arXiv:1606.03437},
year={2016}
}