#!/usr/bin/env python3
import sys
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GuiTeleopNode(Node):
    def __init__(self, initial_topic='/cmd_vel'):
        super().__init__('gui_teleop')
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.max_linear = 0.5   # m/s
        self.max_angular = 1.0  # rad/s
        self.current_topic = initial_topic
        
        self.declare_parameter('cmd_vel_topic', initial_topic)
        resolved_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        
        self.publisher_ = self.create_publisher(Twist, resolved_topic, 10)
        self.timer = self.create_timer(0.1, self.publish_callback)
        self.get_logger().info(f'GUI Teleop Node initialized. Publishing to topic: {resolved_topic}')

    def publish_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.publisher_.publish(msg)

    def set_velocities(self, linear, angular):
        self.linear_vel = float(linear)
        self.angular_vel = float(angular)

    def stop(self):
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    def update_topic(self, new_topic):
        if new_topic == self.current_topic:
            return
        self.get_logger().info(f'Changing active ROS 2 cmd_vel topic from "{self.current_topic}" to "{new_topic}"')
        self.destroy_publisher(self.publisher_)
        self.current_topic = new_topic
        self.publisher_ = self.create_publisher(Twist, new_topic, 10)


class TeleopGui:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        
        # Window setup
        self.root.title("ROAR Rover Agnostic Teleop Panel")
        self.root.geometry("540x530")
        self.root.configure(bg="#1e1e24")
        self.root.resizable(False, False)

        # Style customization
        self.style = ttk.Style()
        self.style.theme_use('default')
        self.style.configure('.', background='#1e1e24', foreground='#e4e4eb')
        self.style.configure('TLabel', font=('Helvetica', 10), background='#1e1e24', foreground='#e4e4eb')
        self.style.configure('TScale', background='#1e1e24')

        # Variables
        self.max_lin_var = tk.DoubleVar(value=self.node.max_linear)
        self.max_ang_var = tk.DoubleVar(value=self.node.max_angular)
        self.topic_var = tk.StringVar(value=self.node.current_topic)

        self.create_widgets()
        self.bind_keys()

    def create_widgets(self):
        # Header Frame
        header = tk.Frame(self.root, bg="#2a2a35", height=60)
        header.pack(fill="x")
        title = tk.Label(header, text="ROAR Multi-Rover Teleop Panel", font=('Helvetica', 14, 'bold'), bg="#2a2a35", fg="#ff6b35")
        title.pack(pady=15)

        # Main Layout: Two Columns
        main_frame = tk.Frame(self.root, bg="#1e1e24")
        main_frame.pack(fill="both", expand=True, padx=20, pady=10)

        left_column = tk.Frame(main_frame, bg="#1e1e24")
        left_column.pack(side="left", fill="both", expand=True)

        right_column = tk.Frame(main_frame, bg="#1e1e24")
        right_column.pack(side="right", fill="both", expand=True, padx=(15, 0))

        # LEFT COLUMN: ROS Topic config & Speed Limits & Buttons
        
        # 1. ROS Topic Configuration Frame
        topic_frame = tk.LabelFrame(left_column, text="ROS Topic Config", bg="#1e1e24", fg="#ff6b35", font=('Helvetica', 10, 'bold'), padx=10, pady=10)
        topic_frame.pack(fill="x", pady=(0, 10))

        tk.Label(topic_frame, text="Active Cmd Vel Topic:", bg="#1e1e24", fg="#8e8e93", font=('Helvetica', 9, 'italic')).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 5))
        
        self.entry_topic = tk.Entry(topic_frame, textvariable=self.topic_var, bg="#121214", fg="#06d6a0", font=('Consolas', 10), bd=1, relief="solid", insertbackground="#ff6b35", highlightthickness=0)
        self.entry_topic.grid(row=1, column=0, sticky="ew", padx=(0, 5), ipady=3)
        
        btn_apply = tk.Button(topic_frame, text="Apply", font=('Helvetica', 9, 'bold'), bg="#ff6b35", fg="#1e1e24", activebackground="#e0531c", bd=0, padx=10, command=self.apply_topic)
        btn_apply.grid(row=1, column=1, sticky="ns")
        
        topic_frame.columnconfigure(0, weight=1)

        # 2. Speed Limits Frame
        speed_frame = tk.LabelFrame(left_column, text="Speed Limits", bg="#1e1e24", fg="#ff6b35", font=('Helvetica', 10, 'bold'), padx=10, pady=10)
        speed_frame.pack(fill="x", pady=(0, 10))

        tk.Label(speed_frame, text="Max Linear (m/s):", bg="#1e1e24", fg="#e4e4eb").grid(row=0, column=0, sticky="w")
        self.lbl_max_lin = tk.Label(speed_frame, text=f"{self.max_lin_var.get():.1f}", bg="#1e1e24", fg="#ff6b35", font=('Helvetica', 10, 'bold'))
        self.lbl_max_lin.grid(row=0, column=2, padx=5)
        self.scale_lin = ttk.Scale(speed_frame, from_=0.1, to=2.0, variable=self.max_lin_var, command=self.update_max_speeds)
        self.scale_lin.grid(row=0, column=1, sticky="ew", pady=5)

        tk.Label(speed_frame, text="Max Angular (rad/s):", bg="#1e1e24", fg="#e4e4eb").grid(row=1, column=0, sticky="w")
        self.lbl_max_ang = tk.Label(speed_frame, text=f"{self.max_ang_var.get():.1f}", bg="#1e1e24", fg="#ff6b35", font=('Helvetica', 10, 'bold'))
        self.lbl_max_ang.grid(row=1, column=2, padx=5)
        self.scale_ang = ttk.Scale(speed_frame, from_=0.2, to=3.0, variable=self.max_ang_var, command=self.update_max_speeds)
        self.scale_ang.grid(row=1, column=1, sticky="ew", pady=5)

        speed_frame.columnconfigure(1, weight=1)

        # 3. Directional Buttons Frame
        btn_frame = tk.LabelFrame(left_column, text="Keyboard & Buttons", bg="#1e1e24", fg="#ff6b35", font=('Helvetica', 10, 'bold'), padx=10, pady=10)
        btn_frame.pack(fill="both", expand=True)

        btn_opts = {"font": ('Helvetica', 10, 'bold'), "bg": "#3a3a4a", "fg": "#e4e4eb", "activebackground": "#ff6b35", "activeforeground": "#1e1e24", "bd": 0, "width": 6, "height": 2}
        
        btn_up = tk.Button(btn_frame, text="▲ (W)", command=lambda: self.drive(1, 0), **btn_opts)
        btn_up.grid(row=0, column=1, pady=5)

        btn_left = tk.Button(btn_frame, text="◀ (A)", command=lambda: self.drive(0, 1), **btn_opts)
        btn_left.grid(row=1, column=0, padx=5)

        btn_stop = tk.Button(btn_frame, text="STOP\n(Space)", font=('Helvetica', 10, 'bold'), bg="#d90429", fg="#ffffff", activebackground="#ef233c", activeforeground="#ffffff", bd=0, width=8, height=2, command=self.stop_rover)
        btn_stop.grid(row=1, column=1, padx=5)

        btn_right = tk.Button(btn_frame, text="▶ (D)", command=lambda: self.drive(0, -1), **btn_opts)
        btn_right.grid(row=1, column=2, padx=5)

        btn_down = tk.Button(btn_frame, text="▼ (S)", command=lambda: self.drive(-1, 0), **btn_opts)
        btn_down.grid(row=2, column=1, pady=5)

        # RIGHT COLUMN: Touchpad Joystick
        joy_frame = tk.LabelFrame(right_column, text="Visual Joystick Pad", bg="#1e1e24", fg="#ff6b35", font=('Helvetica', 10, 'bold'), padx=10, pady=10)
        joy_frame.pack(fill="both", expand=True)

        self.canvas_size = 200
        self.canvas = tk.Canvas(joy_frame, width=self.canvas_size, height=self.canvas_size, bg="#121214", bd=0, highlightthickness=1, highlightbackground="#3a3a4a")
        self.canvas.pack(pady=15, anchor="center")

        # Draw joystick target elements
        self.center = self.canvas_size // 2
        self.canvas.create_oval(self.center - 10, self.center - 10, self.center + 10, self.center + 10, fill="#2a2a35", outline="#ff6b35", tags="joy_knob")
        self.canvas.create_oval(10, 10, self.canvas_size - 10, self.canvas_size - 10, outline="#3a3a4a", width=1)
        self.canvas.create_line(self.center, 0, self.center, self.canvas_size, fill="#222226")
        self.canvas.create_line(0, self.center, self.canvas_size, self.center, fill="#222226")

        # Joystick Event Bindings
        self.canvas.bind("<B1-Motion>", self.process_joystick)
        self.canvas.bind("<ButtonRelease-1>", self.reset_joystick)

        # Instruction info label
        info_label = tk.Label(joy_frame, text="Steer proportionally by dragging.\nRelease mouse to auto-stop.", font=('Helvetica', 8, 'italic'), bg="#1e1e24", fg="#8e8e93")
        info_label.pack(pady=5)

        # Footer Status Panel
        footer = tk.Frame(self.root, bg="#121214", height=40)
        footer.pack(fill="x", side="bottom")
        
        self.lbl_status = tk.Label(footer, text="Linear: 0.00 m/s  |  Angular: 0.00 rad/s", font=('Consolas', 10), bg="#121214", fg="#06d6a0")
        self.lbl_status.pack(pady=10)

    def bind_keys(self):
        self.root.bind("<w>", lambda e: self.drive(1, 0))
        self.root.bind("<s>", lambda e: self.drive(-1, 0))
        self.root.bind("<a>", lambda e: self.drive(0, 1))
        self.root.bind("<d>", lambda e: self.drive(0, -1))
        self.root.bind("<space>", lambda e: self.stop_rover())
        
        self.root.bind("<W>", lambda e: self.drive(1, 0))
        self.root.bind("<S>", lambda e: self.drive(-1, 0))
        self.root.bind("<A>", lambda e: self.drive(0, 1))
        self.root.bind("<D>", lambda e: self.drive(0, -1))

    def apply_topic(self):
        new_topic = self.topic_var.get().strip()
        if not new_topic:
            messagebox.showwarning("Invalid Topic", "Topic name cannot be empty.")
            return
        self.node.update_topic(new_topic)
        self.lbl_status.config(fg="#06d6a0")

    def update_max_speeds(self, *args):
        self.node.max_linear = self.max_lin_var.get()
        self.node.max_angular = self.max_ang_var.get()
        self.lbl_max_lin.config(text=f"{self.node.max_linear:.1f}")
        self.lbl_max_ang.config(text=f"{self.node.max_angular:.1f}")

    def drive(self, lin_dir, ang_dir):
        lin_speed = lin_dir * self.node.max_linear
        img_speed = ang_dir * self.node.max_angular
        self.node.set_velocities(lin_speed, img_speed)
        self.update_status_bar()

    def stop_rover(self):
        self.node.stop()
        self.update_status_bar()

    def process_joystick(self, event):
        x = event.x - self.center
        y = event.y - self.center
        
        max_dist = self.canvas_size // 2 - 10
        dist = (x**2 + y**2)**0.5
        if dist > max_dist:
            x = (x / dist) * max_dist
            y = (y / dist) * max_dist
            dist = max_dist

        self.canvas.delete("joy_knob")
        self.canvas.create_oval(self.center + x - 10, self.center + y - 10, self.center + x + 10, self.center + y + 10, fill="#ff6b35", outline="#ff6b35", tags="joy_knob")

        lin_ratio = -y / max_dist
        ang_ratio = -x / max_dist

        lin_speed = lin_ratio * self.node.max_linear
        ang_speed = ang_ratio * self.node.max_angular

        self.node.set_velocities(lin_speed, ang_speed)
        self.update_status_bar()

    def reset_joystick(self, event):
        self.canvas.delete("joy_knob")
        self.canvas.create_oval(self.center - 10, self.center - 10, self.center + 10, self.center + 10, fill="#2a2a35", outline="#ff6b35", tags="joy_knob")
        self.stop_rover()

    def update_status_bar(self):
        self.lbl_status.config(text=f"Linear: {self.node.linear_vel:+.2f} m/s  |  Angular: {self.node.angular_vel:+.2f} rad/s")


def main(args=None):
    # CLI topic parser: python3 gui_teleop.py --topic=/my_rover/cmd_vel
    initial_topic = '/cmd_vel'
    for arg in sys.argv:
        if arg.startswith('--topic='):
            initial_topic = arg.split('=', 1)[1]

    rclpy.init(args=args)
    node = GuiTeleopNode(initial_topic=initial_topic)

    # Spin ROS 2 node in background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Run Tkinter GUI
    root = tk.Tk()
    gui = TeleopGui(root, node)
    
    try:
        root.mainloop()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
