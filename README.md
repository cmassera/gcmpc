# A Guaranteed Cost Approach to Robust Model Predictive Control of Uncertain Linear Systems

This repository contains complimentary material to the paper "A Guaranteed Cost Approach to Robust Model Predictive Control of Uncertain Linear Systems". 

It contains:
- The paper itself (to be added);
- The source code for the numerical examples;
- Complimentary material with: 
  - Proof of Lemma 3;
  - Discussion on the independence of controllers in Corollary 1.

## Pre-requisites

The source code in this repository requires:

- [MATLAB](http://www.mathworks.com/products/matlab/)

and

- [YALMIP Toolbox](http://users.isy.liu.se/johanl/yalmip/)
- [Gurobi](http://www.gurobi.com)

Installed and referenced in MATLAB's path.

Note: Gurobi has free-licenses for Academia.

## Running the examples

All examples are self contained and can be directly run. For example:

```matlab
>> gcmpc_l2norm
>> plot_results
```

## Citing

This readme will be updated upon paper publication.