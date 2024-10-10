import sys, os
sys.path.append(os.path.dirname(__file__))
POSITION_FILE = os.path.dirname(__file__)+"/current_position.txt"
print(POSITION_FILE)
print(POSITION_FILE)
print(POSITION_FILE)
print(POSITION_FILE)
print(POSITION_FILE)
print(POSITION_FILE)


class TEST:
    def __init__(self):
        
        self.current_position = self.load_current_position()
        print(self.current_position)
        self.save_current_position(67867876876876)
        self.current_position = self.load_current_position()
        print(self.current_position)




    def load_current_position(self):
        """ 파일에서 현재 위치를 불러오는 함수 """
        try:
            with open(POSITION_FILE, 'r') as f:
                position = float(f.read().strip())
                return position
        except FileNotFoundError:
            return 0
        except ValueError:
            return 0

    def save_current_position(self, position):
        """ 파일에 현재 위치를 저장하는 함수 """
        with open(POSITION_FILE, 'w') as f:
            f.write(str(position))


t = TEST()