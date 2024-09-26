import os

def split_log_file(file_path, output_dir, max_size=100 * 1024 * 1024):
    # 检查日志输出目录是否存在，不存在则创建
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 计算每个拆分文件的大小上限
    file_index = 1
    output_file_base_name = "user_algorithm"
    output_file_template = output_file_base_name + "_{:05d}.log"
    
    with open(file_path, 'r') as input_file:
        current_file_size = 0
        current_file_path = os.path.join(output_dir, output_file_template.format(file_index))
        current_output_file = open(current_file_path, 'w')
        
        for line in input_file:
            current_file_size += len(line.encode('utf-8'))  # 计算每行字节大小
            
            if current_file_size > max_size:
                current_output_file.close()  # 关闭当前输出文件
                file_index += 1  # 文件编号递增
                current_file_size = len(line.encode('utf-8'))  # 重置当前文件大小
                current_file_path = os.path.join(output_dir, output_file_template.format(file_index))
                current_output_file = open(current_file_path, 'w')  # 打开新的输出文件
            
            current_output_file.write(line)  # 将当前行写入输出文件
        
        current_output_file.close()  # 关闭最后一个文件

if __name__ == "__main__":
    log_dir = "./log"  # 拆分文件保存的目录
    log_file_path = "./user_algorithm.log"  # 需要拆分的原始文件路径
    
    split_log_file(log_file_path, log_dir)
