import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class SliderApp(Node):
    def __init__(self):
        super().__init__('slider_node')
        self.position = 0
        self.publisher = self.create_publisher(Int16, '/rail/position', 10)

        self.root = tk.Tk()
        self.root.title("Position Publisher")
        self.root.geometry("800x400")  # 화면 크기를 800x400으로 설정

        self.value_label = tk.Label(self.root, text="현재 값: 0", font=("Arial", 16))
        self.value_label.pack(pady=20)

        # 슬라이더 범위를 -5000에서 5000으로 설정
        self.slider = tk.Scale(self.root, from_=-10, to=10, orient=tk.HORIZONTAL, command=self.update_label, length=600)
        self.slider.pack(pady=20)

        # 초기화 버튼 추가
        self.reset_button = tk.Button(self.root, text="초기화", command=self.reset_slider, font=("Arial", 14))
        self.reset_button.pack(pady=20)

        self.publish_button = tk.Button(self.root, text="publish", command=self.publish_value, font=("Arial", 14))
        self.publish_button.pack(pady=20)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def update_label(self, value):
        self.position = int(value) * 360
        self.value_label.config(text=f"현재 값: {self.position}")


    def publish_value(self):
        slider_value = Int16()
        slider_value.data = int(self.position)
        self.publisher.publish(slider_value)  # 슬라이더 값을 퍼블리시

    def reset_slider(self):
        self.slider.set(0)  # 슬라이더 값을 0으로 설정
        self.update_label(0)  # 레이블과 퍼블리셔 업데이트

    def on_closing(self):
        self.root.quit()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    app = SliderApp()
    tk.mainloop()
    rclpy.spin(app)

if __name__ == "__main__":
    main()
