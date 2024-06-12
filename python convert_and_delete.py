from PIL import Image
import os

with Image.open('ckp2.ppm') as img:
    # 保存为PNG格式
    img.save('output.png', 'PNG')

# os.remove('binary.ppm')
