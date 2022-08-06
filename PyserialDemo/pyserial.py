import serial

def main():
    port = input("Please enter the port: ")
    ser = serial.Serial(port, 115200, timeout=2)
    while True:
        if ser.in_waiting >= 0:
            b = ser.read(1).decode("utf-8")
            print(b, end="")

if __name__ == "__main__":
    main()
