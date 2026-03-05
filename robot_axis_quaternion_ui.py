import math
import tkinter as tk
from tkinter import ttk, messagebox


EPS = 1e-10
AXES = ("X", "Y", "Z")
ORDERS = ("XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX")


def quat_normalize(w: float, x: float, y: float, z: float):
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < EPS:
        raise ValueError("四元数长度不能为 0")
    return w / norm, x / norm, y / norm, z / norm


def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def axis_angle_to_quaternion(axis: str, angle_deg: float):
    half = math.radians(angle_deg) / 2.0
    s = math.sin(half)
    c = math.cos(half)

    if axis == "X":
        return c, s, 0.0, 0.0
    if axis == "Y":
        return c, 0.0, s, 0.0
    if axis == "Z":
        return c, 0.0, 0.0, s
    raise ValueError("无效轴")


def compose_relative_axis_rotations(angle_x: float, angle_y: float, angle_z: float, order: str):
    angles = {"X": angle_x, "Y": angle_y, "Z": angle_z}

    q_total = (1.0, 0.0, 0.0, 0.0)
    steps = []
    for axis in order:
        q_step = axis_angle_to_quaternion(axis, angles[axis])
        # Relative/tool-frame sequence: post-multiply each step.
        q_total = quat_mul(q_total, q_step)
        q_total = quat_normalize(*q_total)
        steps.append((axis, angles[axis], q_step, q_total))

    return (*q_total, steps)


def quaternion_to_axis_angle(w: float, x: float, y: float, z: float):
    w, x, y, z = quat_normalize(w, x, y, z)
    w = max(-1.0, min(1.0, w))

    theta = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))

    if s < EPS:
        axis = (1.0, 0.0, 0.0)
    else:
        axis = (x / s, y / s, z / s)

    angle_deg = math.degrees(theta)
    return axis[0], axis[1], axis[2], angle_deg, w, x, y, z


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("机械臂相对轴旋转 <-> 四元数 (wxyz)")
        self.geometry("820x580")
        self.resizable(False, False)
        self._build_ui()

    def _build_ui(self):
        main = ttk.Frame(self, padding=14)
        main.pack(fill="both", expand=True)

        desc = (
            "相对旋转(工具坐标系)会受顺序影响。\n"
            "每步四元数按所选顺序累乘：q_total = q_total ⊗ q_step，输出为 [w, x, y, z]。"
        )
        ttk.Label(main, text=desc, justify="left").pack(anchor="w", pady=(0, 12))

        top = ttk.Frame(main)
        top.pack(fill="x", pady=6)

        rel_frame = ttk.LabelFrame(top, text="相对轴角输入（在对应轴后直接填角度）", padding=10)
        rel_frame.pack(side="left", fill="both", expand=True, padx=(0, 8))

        self.angle_x_var = tk.StringVar(value="0")
        self.angle_y_var = tk.StringVar(value="0")
        self.angle_z_var = tk.StringVar(value="90")
        self.order_var = tk.StringVar(value="XYZ")

        self._row_input(rel_frame, "X 轴角度(度):", self.angle_x_var, 0)
        self._row_input(rel_frame, "Y 轴角度(度):", self.angle_y_var, 1)
        self._row_input(rel_frame, "Z 轴角度(度):", self.angle_z_var, 2)

        ttk.Label(rel_frame, text="旋转顺序:").grid(row=3, column=0, sticky="w", pady=4, padx=(0, 8))
        order_box = ttk.Combobox(rel_frame, textvariable=self.order_var, values=ORDERS, state="readonly", width=16)
        order_box.grid(row=3, column=1, sticky="ew", pady=4)

        ttk.Button(rel_frame, text="相对轴角 -> 四元数(wxyz)", command=self.convert_relative_to_quat).grid(
            row=4, column=0, columnspan=2, sticky="ew", pady=(10, 0)
        )

        quat_frame = ttk.LabelFrame(top, text="四元数输入（wxyz）", padding=10)
        quat_frame.pack(side="left", fill="both", expand=True)

        self.w_var = tk.StringVar(value="0.70710678")
        self.x_var = tk.StringVar(value="0")
        self.y_var = tk.StringVar(value="0")
        self.z_var = tk.StringVar(value="0.70710678")

        self._row_input(quat_frame, "w:", self.w_var, 0)
        self._row_input(quat_frame, "x:", self.x_var, 1)
        self._row_input(quat_frame, "y:", self.y_var, 2)
        self._row_input(quat_frame, "z:", self.z_var, 3)

        ttk.Button(quat_frame, text="四元数(wxyz) -> 轴角", command=self.convert_quat_to_axis).grid(
            row=4, column=0, columnspan=2, sticky="ew", pady=(10, 0)
        )

        result_frame = ttk.LabelFrame(main, text="结果", padding=10)
        result_frame.pack(fill="both", expand=True, pady=(10, 0))

        self.result_text = tk.Text(result_frame, height=16, wrap="word")
        self.result_text.pack(fill="both", expand=True)
        self.result_text.insert(
            "end",
            "说明:\n"
            "1) 该界面按工具坐标系相对轴顺序旋转，顺序不同结果不同。\n"
            "2) 输出四元数格式固定为 [w, x, y, z]。\n"
            "3) 四元数会自动归一化。\n",
        )
        self.result_text.config(state="disabled")

    @staticmethod
    def _row_input(parent, label, variable, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=4, padx=(0, 8))
        ttk.Entry(parent, textvariable=variable, width=18).grid(row=row, column=1, sticky="ew", pady=4)
        parent.grid_columnconfigure(1, weight=1)

    def _write_result(self, text):
        self.result_text.config(state="normal")
        self.result_text.delete("1.0", "end")
        self.result_text.insert("end", text)
        self.result_text.config(state="disabled")

    def convert_relative_to_quat(self):
        try:
            angle_x = float(self.angle_x_var.get())
            angle_y = float(self.angle_y_var.get())
            angle_z = float(self.angle_z_var.get())
            order = self.order_var.get().strip().upper()

            if order not in ORDERS:
                raise ValueError("旋转顺序必须是 XYZ/XZY/YXZ/YZX/ZXY/ZYX 之一")

            w, x, y, z, steps = compose_relative_axis_rotations(angle_x, angle_y, angle_z, order)

            self.w_var.set(f"{w:.10f}")
            self.x_var.set(f"{x:.10f}")
            self.y_var.set(f"{y:.10f}")
            self.z_var.set(f"{z:.10f}")

            lines = [
                "相对轴角 -> 四元数（wxyz）",
                "",
                f"输入角度: X={angle_x:.10f}, Y={angle_y:.10f}, Z={angle_z:.10f} (deg)",
                f"旋转顺序: {order}",
                "",
                "逐步累乘（q_total = q_total ⊗ q_step）:",
            ]

            for i, (axis, angle, q_step, q_total) in enumerate(steps, start=1):
                lines.append(
                    f"{i}. axis={axis}, angle={angle:.10f} -> "
                    f"q_step=[{q_step[0]:.10f}, {q_step[1]:.10f}, {q_step[2]:.10f}, {q_step[3]:.10f}]"
                )
                lines.append(
                    f"   q_total=[{q_total[0]:.10f}, {q_total[1]:.10f}, {q_total[2]:.10f}, {q_total[3]:.10f}]"
                )

            lines.append("")
            lines.append(f"最终 q=[w, x, y, z]=[{w:.10f}, {x:.10f}, {y:.10f}, {z:.10f}]")
            self._write_result("\n".join(lines))
        except ValueError as e:
            messagebox.showerror("输入错误", str(e))
        except Exception:
            messagebox.showerror("输入错误", "请检查输入是否为有效数字")

    def convert_quat_to_axis(self):
        try:
            w = float(self.w_var.get())
            x = float(self.x_var.get())
            y = float(self.y_var.get())
            z = float(self.z_var.get())

            ax, ay, az, angle, wn, xn, yn, zn = quaternion_to_axis_angle(w, x, y, z)

            self._write_result(
                "四元数（wxyz） -> 轴角\n\n"
                f"归一化四元数 q: [{wn:.10f}, {xn:.10f}, {yn:.10f}, {zn:.10f}]\n\n"
                f"单位轴 (工具坐标系): [{ax:.10f}, {ay:.10f}, {az:.10f}]\n"
                f"角度 θ: {angle:.10f} deg"
            )
        except ValueError as e:
            messagebox.showerror("输入错误", str(e))
        except Exception:
            messagebox.showerror("输入错误", "请检查输入是否为有效数字")


if __name__ == "__main__":
    app = App()
    app.mainloop()
