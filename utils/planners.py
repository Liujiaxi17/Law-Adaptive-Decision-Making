# import os
from cmath import sqrt
import numpy as np


class BackupPlanner():
    def __init__(self):
        '''
            params init
            Backup planner will generate lane-change motivation when:
            -p*a* (d_0+T*v+v* (v_f - v)/(2*sqrt(a*b)) )^2 / d^2 > a_thre + a_lanebias (+ a_lawbias)
            law: d > d_thre
        '''
        self.p = 0.5
        self.p = 0.1
        self.a = 1.4
        self.d_0 = 2.0
        self.T = 1.5
        self.b = 2.0

        self.a_thre = 0.1
        self.a_lane_bias = -0.32
        self.a_law_bias = 0.12
        self.a_lane_bias = -0.45
        self.a_law_bias = 0.25

    def plan(self, v, v_f, d, d_thre):
        '''
            return: 
                0: no change
                1: change
        '''

        not_law_compliance = int(d < d_thre)


        motivation = -self.p * self.a * (self.d_0 + self.T * v + v * (v_f - v) / (2 * ((self.a * self.b)**0.5)))**2 / (d**2) 
        # print(motivation)
        
        rule_action = int(motivation > self.a_thre + self.a_lane_bias + self.a_law_bias * not_law_compliance)
        
        if not_law_compliance and rule_action:
            print("rule breaks law!, rule_action is ", rule_action,". motivation is ", motivation)

        return rule_action, motivation

    def plan_without_law(self, v, v_f, d):
        '''
            return: 
                0: no change
                1: change
        '''
        motivation = -self.p * self.a * (self.d_0 + self.T * v + v * (v_f - v) / (2 * ((self.a * self.b)**0.5)))**2 / (d**2) 
        # print("motivation", motivation)
        
        rule_action = int(motivation > self.a_thre + self.a_lane_bias)
        
        return rule_action, motivation