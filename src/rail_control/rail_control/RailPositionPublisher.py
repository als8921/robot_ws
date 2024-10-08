import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class Int16Publisher(Node):
    def __init__(self):
        super().__init__('int16_publisher_node')
        self.pos_publisher = self.create_publisher(String, '/rcs/rail_refpos', 10)
        self.emg_publisher = self.create_publisher(Bool, '/rcs/rail_emg', 10)
        self.cali_publisher = self.create_publisher(Bool, '/rcs/rail_calib',10)


        self.root = tk.Tk()
        self.root.title("Position Publish")
        self.root.geometry("400x300")  # 화면 크기를 400x300으로 설정

        # 레이블
        self.value_label = tk.Label(self.root, text="Position", font=("Arial", 14))
        self.value_label.grid(row=0, column=0, columnspan=2, pady=20, sticky='nsew')

        # 입력 필드
        self.int_entry = tk.Entry(self.root, font=("Arial", 14))
        self.int_entry.grid(row=1, column=0, columnspan=2, pady=5, sticky='nsew')

        # 퍼블리시 버튼
        self.publish_button = tk.Button(self.root, text="Publish", command=self.publish_value, font=("Arial", 14))
        self.publish_button.grid(row=2, column=0, columnspan=2, pady=10, sticky='nsew')

        # 위험 버튼
        self.warning_active = False
        self.warning_button = tk.Button(self.root, text="위험 버튼", command=self.toggle_warning, bg='green', font=("Arial", 14))
        self.warning_button.grid(row=3, column=0, pady=10, sticky='ew')

        # 홈 버튼
        self.homing_button = tk.Button(self.root, text="Homing", command=self.publish_homing, font=("Arial", 14))
        self.homing_button.grid(row=3, column=1, columnspan=2, pady=20, sticky='ew')

        # 그리드 행과 열의 비율 설정
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_rowconfigure(3, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def publish_value(self):
        try:
            string_data = String()
            string_data.data = str(self.int_entry.get())  # 입력된 값을 Int16로 변환
            self.pos_publisher.publish(string_data)  # 메시지 퍼블리시
            print(f"퍼블리시된 값: {string_data.data}")  # 콘솔에 출력
        except ValueError:
            print("유효한 값 입력하세요.")

    def toggle_warning(self):
        self.warning_active = not self.warning_active
        if self.warning_active:
            self.warning_button.config(bg='red')
            self.emg_publisher.publish(Bool(data = True))  # Bool 메시지 퍼블리시
        else:
            self.warning_button.config(bg='green')
            self.emg_publisher.publish(Bool(data = False))  # Bool 메시지 퍼블리시


    def publish_homing(self):
        self.cali_publisher.publish(Bool(data = True))  # 홈 버튼 클릭 시 True 퍼블리시

    def on_closing(self):
        self.root.quit()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    app = Int16Publisher()
    try:
        tk.mainloop()
    finally:
        app.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
