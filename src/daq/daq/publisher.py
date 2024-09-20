import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SliderApp(Node):
    def __init__(self):
        super().__init__('slider_node')
        self.publisher = self.create_publisher(Float32, 'desired_position', 10)

        self.root = tk.Tk()
        self.root.title("슬라이더 바 예제")
        self.root.geometry("800x400")  # 화면 크기를 800x400으로 설정

        self.value_label = tk.Label(self.root, text="현재 값: 0", font=("Arial", 16))
        self.value_label.pack(pady=20)

        # 슬라이더 범위를 -5000에서 5000으로 설정
        self.slider = tk.Scale(self.root, from_=-5000, to=5000, orient=tk.HORIZONTAL, command=self.update_label, length=600)
        self.slider.pack(pady=20)

        # 초기화 버튼 추가
        self.reset_button = tk.Button(self.root, text="초기화", command=self.reset_slider, font=("Arial", 14))
        self.reset_button.pack(pady=20)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def update_label(self, value):
        self.value_label.config(text=f"현재 값: {value}")
        slider_value = Float32()
        slider_value.data = float(value)
        self.publisher.publish(slider_value)  # 슬라이더 값을 퍼블리시
        print(f"슬라이더 값: {value}")  # 슬라이더 값을 콘솔에 출력

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
