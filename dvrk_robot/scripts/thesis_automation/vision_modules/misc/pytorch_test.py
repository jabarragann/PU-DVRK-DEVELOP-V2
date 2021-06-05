#!/usr/bin/env python

import torch
x = torch.rand(5, 3)
print(x)


print("CUDA available ",torch.cuda.is_available())