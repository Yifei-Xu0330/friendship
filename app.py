import math
from dataclasses import dataclass

import streamlit as st


# =====================
# 计算核心（和之前 Python 版一致）
# =====================
@dataclass
class SectionResult:
    mode: str               # 'single', 'double', 'check'
    h0: float               # 有效高度 mm
    alpha_s: float          # 相对弯矩系数
    alpha_s_max: float      # 单筋极限相对弯矩
    x: float                # 受压区高度 mm
    As: float               # 受拉钢筋面积 mm²
    As_prime: float         # 受压钢筋面积 mm²
    M_capacity: float       # 弯矩承载力 kN·m
    extra: dict             # 其他信息


def _calc_effective_depth(h: float, a_s: float) -> float:
    return h - a_s


def _alpha_s(M: float, b: float, h0: float, f_c: float, alpha1: float) -> float:
    M_Nmm = M * 1e6  # kN·m -> N·mm
    return M_Nmm / (alpha1 * f_c * b * h0 ** 2)


def _alpha_s_max(xi_b: float) -> float:
    return xi_b * (1 - 0.5 * xi_b)


def _solve_xi(alpha_s: float) -> float:
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


# =====================
# Streamlit 界面
# =====================

st.set_page_config(page_title="双筋矩形截面设计", layout="centered")

st.title("双筋矩形截面正截面受弯设计 / 验算工具")

st.markdown(
    """
**单位约定：**

- b, h, aₛ, a′ₛ：mm  
- 弯矩 M：kN·m  
- f：MPa = N/mm²  
"""
)

mode = st.radio("选择模式", ["按弯矩设计配筋", "已知配筋验算承载力"])

st.subheader("截面与材料参数")

col1, col2 = st.columns(2)
with col1:
    b = st.number_input("b（截面宽 mm）", value=250.0, step=10.0)
    h = st.number_input("h（截面高 mm）", value=500.0, step=10.0)
    a_s = st.number_input("aₛ（受拉钢筋合力点至受压缘 mm）", value=60.0, step=5.0)
    a_s_prime = st.number_input("a′ₛ（受压钢筋合力点至受压缘 mm）", value=40.0, step=5.0)
with col2:
    f_c = st.number_input("f_c（混凝土设计值 MPa）", value=14.3, step=0.1)
    f_y = st.number_input("f_y（受拉钢筋 MPa）", value=360.0, step=10.0)
    f_y_prime = st.number_input("f′_y（受压钢筋 MPa）", value=360.0, step=10.0)
    alpha1 = st.number_input("α₁（等效应力块系数）", value=1.0, step=0.01)
xi_b = st.number_input("ξ_b（极限受压区高度系数）", value=0.518, step=0.001)


if mode == "按弯矩设计配筋":
    st.subheader("荷载参数")
    M = st.number_input("弯矩设计值 M（kN·m）", value=325.0, step=10.0)

    if st.button("计算配筋"):
        try:
            res = design_doubly_reinforced(
                b=b,
                h=h,
                a_s=a_s,
                a_s_prime=a_s_prime,
                M=M,
                f_c=f_c,
                f_y=f_y,
                f_y_prime=f_y_prime,
                alpha1=alpha1,
                xi_b=xi_b,
            )
        except Exception as e:
            st.error(f"计算出错：{e}")
        else:
            st.success(f"计算完成，模式：{'单筋截面' if res.mode=='single' else '双筋截面'}")

            st.markdown("### 关键中间参数")
            st.write(f"h₀ = {res.h0:.2f} mm")
            st.write(f"αₛ = {res.alpha_s:.4f}")
            st.write(f"αₛ,max = {res.alpha_s_max:.4f}")
            st.write(f"ξ = {res.extra.get('xi', float('nan')):.4f}")
            st.write(f"x = {res.x:.2f} mm")

            st.markdown("### 配筋结果")
            st.write(f"A_s（受拉钢筋面积） = **{res.As:.1f} mm²**")
            if res.mode == "double":
                st.write(f"A′_s（受压钢筋面积） = **{res.As_prime:.1f} mm²**")
            else:
                st.write("A′_s（受压钢筋面积） = 0（单筋截面）")

            st.markdown("### 弯矩承载力")
            st.write(f"M_u ≈ **{res.M_capacity:.2f} kN·m**")

            if res.mode == "double":
                st.markdown("### 分项弯矩（仅供查看计算过程）")
                st.write(f"M_c（混凝土贡献） ≈ {res.extra['M_c_kNm']:.2f} kN·m")
                st.write(f"ΔM（钢筋对偶承担） ≈ {res.extra['delta_M_kNm']:.2f} kN·m")

else:
    st.subheader("已知配筋参数（用于验算）")
    col3, col4 = st.columns(2)
    with col3:
        A_s = st.number_input("A_s（受拉钢筋面积 mm²）", value=2600.0, step=50.0)
    with col4:
        A_s_prime = st.number_input("A′_s（受压钢筋面积 mm²）", value=800.0, step=50.0)

    if st.button("验算承载力"):
        try:
            res = check_section_capacity(
                b=b,
                h=h,
                a_s=a_s,
                a_s_prime=a_s_prime,
                A_s=A_s,
                A_s_prime=A_s_prime,
                f_c=f_c,
                f_y=f_y,
                f_y_prime=f_y_prime,
                alpha1=alpha1,
                xi_b=xi_b,
            )
        except Exception as e:
            st.error(f"计算出错：{e}")
        else:
            st.success("验算完成")

            st.markdown("### 中间参数")
            st.write(f"h₀ = {res.h0:.2f} mm")
            st.write(f"ξ = {res.extra.get('xi', float('nan')):.4f}")
            st.write(f"x = {res.x:.2f} mm")
            st.write(f"αₛ(由 M_u 反算) = {res.alpha_s:.4f}")
            st.write(f"αₛ,max = {res.alpha_s_max:.4f}")

            st.markdown("### 弯矩承载力")
            st.write(f"M_u ≈ **{res.M_capacity:.2f} kN·m**")

            st.markdown("### 内力平衡情况（仅做参考）")
            st.write(f"C_c（混凝土压力） = {res.extra['C_c']:.1f} N")
            st.write(f"C_s（受压钢筋压力） = {res.extra['C_s']:.1f} N")
            st.write(f"T_s（受拉钢筋拉力） = {res.extra['T_s']:.1f} N")
            st.write(f"C_total = {res.extra['C_total']:.1f} N")
            st.write(
                f"T_s - C_total = {res.extra['unbalanced_force']:.1f} N "
                "(接近 0 说明平衡较好)"
            )

st.markdown("---")
st.caption("简化计算假定：受压钢筋屈服、受压区取 ξ = ξ_b，适用于教材例题级别的设计 / 验算练习。")
