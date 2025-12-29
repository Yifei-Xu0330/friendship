#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from dataclasses import dataclass


@dataclass
class SectionResult:
    mode: str               # 'single', 'double', 'check'
    h0: float               # 有效高度 mm
    alpha_s: float          # 相对弯矩系数
    alpha_s_max: float      # 单筋极限相对弯矩
    x: float                # 受压区高度 mm
    As: float               # 受拉钢筋面积 mm²
    As_prime: float         # 受压钢筋面积 mm²
    M_capacity: float       # 计算得到的弯矩承载力 kN·m
    extra: dict             # 其他辅助信息


def _calc_effective_depth(h: float, a_s: float) -> float:
    return h - a_s


def _alpha_s(M: float, b: float, h0: float, f_c: float, alpha1: float) -> float:
    M_Nmm = M * 1e6  # kN·m -> N·mm
    return M_Nmm / (alpha1 * f_c * b * h0 ** 2)


def _alpha_s_max(xi_b: float) -> float:
    return xi_b * (1 - 0.5 * xi_b)


def _solve_xi(alpha_s: float) -> float:
    # 解 0.5 xi^2 - xi + alpha_s = 0
    if alpha_s < 0 or alpha_s > 0.5:
        raise ValueError("alpha_s 超出单筋截面适用范围 (0 ~ 0.5)")
    disc = 1.0 - 2.0 * alpha_s
    if disc < 0:
        disc = 0.0
    return 1.0 - math.sqrt(disc)


def design_doubly_reinforced(
    b: float,
    h: float,
    a_s: float,
    a_s_prime: float,
    M: float,
    f_c: float,
    f_y: float,
    f_y_prime: float,
    alpha1: float = 1.0,
    xi_b: float = 0.518,
) -> SectionResult:

    h0 = _calc_effective_depth(h, a_s)
    alpha_s_val = _alpha_s(M, b, h0, f_c, alpha1)
    alpha_s_lim = _alpha_s_max(xi_b)

    # 单筋情况
    if alpha_s_val <= alpha_s_lim:
        xi = _solve_xi(alpha_s_val)
        x = xi * h0
        As = alpha1 * f_c * b * x / f_y
        M_capacity = alpha1 * f_c * b * x * (h0 - 0.5 * x) / 1e6

        return SectionResult(
            mode="single",
            h0=h0,
            alpha_s=alpha_s_val,
            alpha_s_max=alpha_s_lim,
            x=x,
            As=As,
            As_prime=0.0,
            M_capacity=M_capacity,
            extra={"xi": xi},
        )

    # 双筋情况
    xi = xi_b
    x = xi * h0

    Mc = alpha1 * f_c * b * x * (h0 - 0.5 * x)  # N·mm
    M_total_Nmm = M * 1e6
    delta_M = max(M_total_Nmm - Mc, 0.0)

    lever_arm_cs = h0 - a_s_prime
    As_prime = delta_M / (f_y_prime * lever_arm_cs)

    As1 = alpha1 * f_c * b * x / f_y
    As = As1 + As_prime * f_y_prime / f_y

    M_capacity = Mc + f_y_prime * As_prime * lever_arm_cs
    M_capacity_kNm = M_capacity / 1e6

    return SectionResult(
        mode="double",
        h0=h0,
        alpha_s=alpha_s_val,
        alpha_s_max=alpha_s_lim,
        x=x,
        As=As,
        As_prime=As_prime,
        M_capacity=M_capacity_kNm,
        extra={
            "xi": xi,
            "M_c_kNm": Mc / 1e6,
            "delta_M_kNm": delta_M / 1e6,
        },
    )


def check_section_capacity(
    b: float,
    h: float,
    a_s: float,
    a_s_prime: float,
    A_s: float,
    A_s_prime: float,
    f_c: float,
    f_y: float,
    f_y_prime: float,
    alpha1: float = 1.0,
    xi_b: float = 0.518,
) -> SectionResult:

    h0 = _calc_effective_depth(h, a_s)
    xi = xi_b
    x = xi * h0

    C_c = alpha1 * f_c * b * x
    C_s = f_y_prime * A_s_prime
    T_s = f_y * A_s
    C_total = C_c + C_s

    M_c = C_c * (h0 - 0.5 * x)
    M_cs = C_s * (h0 - a_s_prime)
    M_capacity = M_c + M_cs
    M_capacity_kNm = M_capacity / 1e6

    alpha_s_val = _alpha_s(M_capacity_kNm, b, h0, f_c, alpha1)
    alpha_s_lim = _alpha_s_max(xi_b)

    extra = {
        "C_c": C_c,
        "C_s": C_s,
        "T_s": T_s,
        "C_total": C_total,
        "unbalanced_force": T_s - C_total,
        "M_c_kNm": M_c / 1e6,
        "M_cs_kNm": M_cs / 1e6,
        "xi": xi,
    }

    return SectionResult(
        mode="check",
        h0=h0,
        alpha_s=alpha_s_val,
        alpha_s_max=alpha_s_lim,
        x=x,
        As=A_s,
        As_prime=A_s_prime,
        M_capacity=M_capacity_kNm,
        extra=extra,
    )


if __name__ == "__main__":
    print("=== 设计示例 ===")
    example = design_doubly_reinforced(
        b=250,
        h=500,
        a_s=62.5,
        a_s_prime=40,
        M=325,
        f_c=14.3,
        f_y=360,
        f_y_prime=360,
        alpha1=1.0,
        xi_b=0.518,
    )
    print(example)

    print("\n=== 验算示例 ===")
    check = check_section_capacity(
        b=250,
        h=500,
        a_s=62.5,
        a_s_prime=40,
        A_s=2600,
        A_s_prime=800,
        f_c=14.3,
        f_y=360,
        f_y_prime=360,
        alpha1=1.0,
        xi_b=0.518,
    )
    print(check)
