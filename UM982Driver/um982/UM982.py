from pyproj import CRS, Transformer
import threading
import serial
import time
import math
import base64
import socket
from pyrtcm import RTCMMessage


def crc_table():
    table = []
    for i in range(256):
        crc = i
        for j in range(8, 0, -1):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
        table.append(crc)
    return table

NMEA_EXPEND_CRC_TABLE = crc_table()


def nmea_expend_crc(nmea_expend_sentence):
    def calculate_crc32(data):
        crc = 0
        for byte in data:
            crc = NMEA_EXPEND_CRC_TABLE[(crc ^ byte) & 0xFF] ^ (crc >> 8)
        return crc & 0xFFFFFFFF

    try:
        sentence, crc = nmea_expend_sentence[1:].split("*")
        crc = crc[:8]
    except:
        return False
    calculated_crc = calculate_crc32(sentence.encode())
    return crc.lower() == format(calculated_crc, '08x')

def nmea_crc(nmea_sentence):
    # 移除起始的'$'和'*'及之后的校验和部分
    try:
        sentence, crc = nmea_sentence[1:].split("*")
        crc = crc[:2]
    except:
        return False
    calculated_checksum = 0
    # 对字符串中的每个字符进行异或运算
    for char in sentence:
        calculated_checksum ^= ord(char)
    # 将计算得到的校验和转换为十六进制格式，并大写
    calculated_checksum_hex = format(calculated_checksum, 'X')
    # 校验和比较
    return calculated_checksum_hex.zfill(2) == crc.upper()

def msg_seperate(msg:str):
    return msg[1:msg.find('*')].split(',')

def PVTSLN_solver(msg:str):
    parts = msg_seperate(msg)
    bestpos_hgt    = float(parts[3+7])          # 海拔高，单位：米
    bestpos_lat    = float(parts[4+7])          # 纬度，单位：度 （输出小数点后 11 位）
    bestpos_lon    = float(parts[5+7])          # 经度，单位：度 （输出小数点后 11 位）
    bestpos_hgtstd = float(parts[6+7])          # 高程标准差，单位：米
    bestpos_latstd = float(parts[7+7])          # 纬度标准差，单位：米
    bestpos_lonstd = float(parts[8+7])          # 经度标准差，单位：米
    fix = (bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd)
    return fix


def GNHPR_solver(msg:str):
    parts = msg_seperate(msg)
    heading = float(parts[3-1])
    pitch   = float(parts[4-1])
    roll    = float(parts[5-1])
    orientation = (heading, pitch, roll)
    return orientation

def UNIHEADING_solver(msg:str):
    parts = msg_seperate(msg)
    heading = float(parts[5+7])
    hdgstddev = float(parts[8+7])
    uniheading_msg = (heading, hdgstddev)
    return uniheading_msg

def BESTNAV_solver(msg:str):
    parts = msg_seperate(msg)
    vel_hor_std = float(parts[-1])  # 水平速度标准差，单位 m/s
    vel_ver_std = float(parts[-2])  # 高程速度标准差，单位 m/s
    vel_ver     = float(parts[-3])  # 垂直速度， m/s，正值表示高度增加（向上），负值表示高度下降（向下）
    vel_heading = float(parts[-4])  # 相对于真北的实际对地运动方向（相对地面轨迹）， deg
    vel_hor     = float(parts[-5])  # 对地水平速度， m/s
    vel_north   = vel_hor * math.cos(math.radians(vel_heading))     # 分解得到北方向速度
    vel_east    = vel_hor * math.sin(math.radians(vel_heading))     # 分解得到东方向速度
    return (vel_east, vel_north, vel_ver, vel_hor_std, vel_hor_std, vel_ver_std)



def create_utm_trans(lat, lon):
    """构建转换器，用于将WGS84地理坐标系下的点转换为UTM坐标系下的点。

    Args:
        lon (float): 点的经度。
        lat (float): 点的纬度。

    Returns:
        transformer: 转换器
    """
    # UTM区号是根据经度确定的，从-180度开始每6度一个区间。
    zone_number            = int((lon + 180) / 6) + 1
    # 北半球是赤道（纬度0度）以上的区域。
    isnorth                = lat >= 0
    # 定义WGS84坐标系
    wgs84_crs              = CRS("epsg:4326")
    # 根据是否位于北半球，选择合适的UTM EPSG代码
    utm_crs_str            = f"epsg:326{zone_number}" if isnorth else f"epsg:327{zone_number}"
    utm_crs                = CRS(utm_crs_str)
    # 创建坐标转换器，从WGS84转换到UTM
    transformer            = Transformer.from_crs(wgs84_crs, utm_crs, always_xy=True)
    return transformer


def utm_trans(transformer, lon, lat):
    """将WGS84地理坐标系下的点转换为UTM坐标系下的点。

    Args:
        transformer: 转换器
        lon (float): 点的经度。
        lat (float): 点的纬度。

    Returns:
        tuple: 一个元组，包含转换后的UTM坐标系下的x（东坐标）和y（北坐标）。
    """
    # 进行坐标转换
    utm_x, utm_y           = transformer.transform(lon, lat)
    return (utm_x, utm_y)

def string_to_base64(input_string):
    input_bytes = input_string.encode("utf-8")
    base64_encoded = base64.b64encode(input_bytes)
    return base64_encoded.decode("utf-8")

def connect_ntrip(host, port, mountpoint, username, password):
    """Connect to NTRIP caster and stream RTCM data."""
    auth = string_to_base64(f"{username}:{password}")
    authorization = f"Authorization: Basic {auth}\r\n"
    url = f"/{mountpoint}\r\n"
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    request = f"GET {url}HTTP/1.1\r\nUser-Agent: NTRIP GNSSInternetRadio/1.4.10\r\nAccept: */*\r\nConnection: close\r\n{authorization}\r\n"
    sock.sendall(request.encode())

    response = sock.recv(1024)

    if b"ICY 200 OK" not in response:
        raise ConnectionError("Failed to connect to NTRIP caster")
    return sock


class UM982Serial():
    def __init__(self, port, band, ntrip_host, ntrip_port, mountpoint, username, password):
        super().__init__()
        # 打开串口
        self.ser            = serial.Serial(port, band)
        # 设置运行标志位
        self.isRUN          = True
        # 数据
        self.fix            = None   # sensor_msgs/NavSatFix 所需要的数据
        self.orientation    = None   # 航向角
        self.vel            = None   # 速度
        self.utmpos         = None
        self.uniheading     = None
        self.ntrip_socket   = None
        self.last_gga       = None
        # 读初始数据
        try:
            self.ntrip_socket = connect_ntrip(ntrip_host, ntrip_port, mountpoint, username, password)
            print("Connected to NTRIP Caster.")
        except Exception as e:
            print(f"Error: {e}")
            return
        for i in range(10):
            self.read_frame()

    def forward_to_serial(self, sock, gga_message):
        """Read RTCM data from NTRIP and forward to serial port."""
        # with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
        sock.sendall(gga_message.encode())
        data, _ = sock.recvfrom(1024)
        if data:
            rtcm_message = RTCMMessage(data)
            if rtcm_message:
                self.ser.write(data)

    def read_frame(self):
        frame = self.ser.readline().decode('utf-8')
        if frame.startswith("$GNGGA"):
            self.last_gga = frame
        if frame.startswith("#PVTSLNA") and nmea_expend_crc(frame):
            self.fix = PVTSLN_solver(frame)
        elif frame.startswith("$GNHPR") and nmea_crc(frame):
            self.orientation = GNHPR_solver(frame)
        elif frame.startswith("#BESTNAVA") and nmea_expend_crc(frame):
            self.vel = BESTNAV_solver(frame)
        elif frame.startswith("#UNIHEADINGA") and nmea_expend_crc(frame):
            self.uniheading = UNIHEADING_solver(frame)
    
    def send_rtcm(self):
        if self.ntrip_socket is not None and self.last_gga is not None:
            self.forward_to_serial(self.ntrip_socket, self.last_gga)


if __name__ == "__main__":
    um982 = UM982Serial("/dev/rtk1", 921600)
    um982.start()

