# Copyright (c) 2021 fortiss GmbH
# 
# Authors: Klemens Esterle and Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np
import scipy.io

def print_poly(nr_reg, data, name):
    outstr = 'const std::vector<double> ' + name + ' = {'

    # implicit transpose!!!
    for j in range(0,3):
        for i in range(0,nr_reg):
            outstr += str(data[0][0][0][i][j][0]) + ', '

    outstr = outstr[:-2] #tailing comma
    outstr += '};'
    outstr += '\n\n'
    print(outstr)


def main():
    nr_reg = 16

    # orientation polys
    matfile = 'data/polynoms_from_fitting_'+str(nr_reg)+'.mat'
    mat = scipy.io.loadmat(matfile)

    sin_ub = mat['lin_result']['poly_sin_ub']
    sin_lb = mat['lin_result']['poly_sin_lb']
    cos_ub = mat['lin_result']['poly_cos_ub']
    cos_lb = mat['lin_result']['poly_cos_lb']

    print_poly(nr_reg, sin_ub, 'POLY_SINT_UB_'+str(nr_reg))
    print_poly(nr_reg, sin_lb, 'POLY_SINT_LB_'+str(nr_reg))
    print_poly(nr_reg, cos_ub, 'POLY_COSS_UB_'+str(nr_reg))
    print_poly(nr_reg, cos_lb, 'POLY_COSS_LB_'+str(nr_reg))

    # curvature polys
    matfile = 'data/poly_curvature_'+str(nr_reg)+'.mat'
    mat = scipy.io.loadmat(matfile)

    kappa_lb = mat['lin_curvature_result']['poly_z_prime_lb']
    kappa_ub = mat['lin_curvature_result']['poly_z_prime_ub']

    print_poly(nr_reg, kappa_ub, 'POLY_KAPPA_AX_MAX_'+str(nr_reg))
    print_poly(nr_reg, kappa_lb, 'POLY_KAPPA_AX_MIN_'+str(nr_reg))





if __name__ == "__main__":
    main()




