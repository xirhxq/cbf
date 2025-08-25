#!/usr/bin/env python3
"""
数据分析脚本，用于提取和分析CBF数据文件中的特定参数
"""

import json
import numpy as np
import argparse
import sys
import os

def load_data(file_path):
    """加载JSON数据文件"""
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
        return data
    except FileNotFoundError:
        print(f"错误: 找不到文件 {file_path}")
        sys.exit(1)
    except json.JSONDecodeError:
        print(f"错误: 文件 {file_path} 不是有效的JSON格式")
        sys.exit(1)

def extract_parameters(data):
    """从数据中提取特定参数"""
    results = {
        "runtime": [],
        "position": [],
        "velocity": [],
        "battery": [],
        "cbf_slack": [],
        "cbf_no_slack": []
    }
    
    for frame in data["state"]:
        robot = frame["robots"][0]  # 假设只有一个机器人
        
        # 提取时间
        results["runtime"].append(frame["runtime"])
        
        # 提取位置
        results["position"].append({
            "x": robot["state"]["x"],
            "y": robot["state"]["y"]
        })
        
        # 提取电池电量
        results["battery"].append(robot["state"]["battery"])
        
        # 提取控制输入（速度）
        if "opt" in robot and "result" in robot["opt"]:
            results["velocity"].append({
                "vx": robot["opt"]["result"]["vx"],
                "vy": robot["opt"]["result"]["vy"]
            })
        else:
            results["velocity"].append({
                "vx": 0.0,
                "vy": 0.0
            })
        
        # 提取CBF值
        if "cbfSlack" in robot:
            results["cbf_slack"].append(robot["cbfSlack"])
        else:
            results["cbf_slack"].append({})
            
        if "cbfNoSlack" in robot:
            results["cbf_no_slack"].append(robot["cbfNoSlack"])
        else:
            results["cbf_no_slack"].append({})
    
    return results

def analyze_parameters(params):
    """分析提取的参数"""
    print("=== 参数分析结果 ===")
    
    # 分析电池电量
    battery_values = np.array(params["battery"])
    print(f"电池电量范围: {battery_values.min():.2f} mV - {battery_values.max():.2f} mV")
    print(f"平均电池电量: {battery_values.mean():.2f} mV")
    print(f"电池消耗: {battery_values[0] - battery_values[-1]:.2f} mV")
    
    # 分析位置
    x_positions = np.array([pos["x"] for pos in params["position"]])
    y_positions = np.array([pos["y"] for pos in params["position"]])
    total_distance = np.sqrt((x_positions[-1] - x_positions[0])**2 + (y_positions[-1] - y_positions[0])**2)
    print(f"总位移: {total_distance:.2f} m")
    
    # 分析速度
    vx_values = np.array([vel["vx"] for vel in params["velocity"]])
    vy_values = np.array([vel["vy"] for vel in params["velocity"]])
    speed_values = np.sqrt(vx_values**2 + vy_values**2)
    print(f"速度范围: {speed_values.min():.4f} m/s - {speed_values.max():.4f} m/s")
    print(f"平均速度: {speed_values.mean():.4f} m/s")
    
    # 分析CBF值
    print("\n=== CBF值分析 ===")
    
    # 分析有松弛变量的CBF (应该在-100到0之间)
    if params["cbf_slack"]:
        print("有松弛变量的CBF (期望范围: -100 到 0):")
        # 获取所有CBF名称
        cbf_names_slack = set()
        for cbf_dict in params["cbf_slack"]:
            cbf_names_slack.update(cbf_dict.keys())
        
        # 分析每个CBF
        for cbf_name in cbf_names_slack:
            cbf_values = [cbf_dict.get(cbf_name, np.nan) for cbf_dict in params["cbf_slack"]]
            cbf_values = np.array(cbf_values)
            # 过滤掉NaN值
            valid_values = cbf_values[~np.isnan(cbf_values)]
            if len(valid_values) > 0:
                print(f"  {cbf_name}:")
                print(f"    范围: {valid_values.min():.4f} - {valid_values.max():.4f}")
                print(f"    平均值: {valid_values.mean():.4f}")
                print(f"    标准差: {valid_values.std():.4f}")
    
    # 分析无松弛变量的CBF (应该在0到100之间)
    if params["cbf_no_slack"]:
        print("\n无松弛变量的CBF (期望范围: 0 到 100):")
        # 获取所有CBF名称
        cbf_names_no_slack = set()
        for cbf_dict in params["cbf_no_slack"]:
            cbf_names_no_slack.update(cbf_dict.keys())
        
        # 分析每个CBF
        for cbf_name in cbf_names_no_slack:
            cbf_values = [cbf_dict.get(cbf_name, np.nan) for cbf_dict in params["cbf_no_slack"]]
            cbf_values = np.array(cbf_values)
            # 过滤掉NaN值
            valid_values = cbf_values[~np.isnan(cbf_values)]
            if len(valid_values) > 0:
                print(f"  {cbf_name}:")
                print(f"    范围: {valid_values.min():.4f} - {valid_values.max():.4f}")
                print(f"    平均值: {valid_values.mean():.4f}")
                print(f"    标准差: {valid_values.std():.4f}")
    
    # 检查CBF值是否在期望范围内
    print("\n=== CBF范围检查 ===")
    
    # 检查有松弛变量的CBF
    if params["cbf_slack"]:
        for cbf_name in cbf_names_slack:
            cbf_values = [cbf_dict.get(cbf_name, np.nan) for cbf_dict in params["cbf_slack"]]
            cbf_values = np.array(cbf_values)
            valid_values = cbf_values[~np.isnan(cbf_values)]
            if len(valid_values) > 0:
                in_range = np.sum((valid_values >= -100) & (valid_values <= 0))
                total_valid = len(valid_values)
                percentage = (in_range / total_valid) * 100
                print(f"{cbf_name} (有松弛变量): {percentage:.1f}% 的值在 [-100, 0] 范围内")
    
    # 检查无松弛变量的CBF
    if params["cbf_no_slack"]:
        for cbf_name in cbf_names_no_slack:
            cbf_values = [cbf_dict.get(cbf_name, np.nan) for cbf_dict in params["cbf_no_slack"]]
            cbf_values = np.array(cbf_values)
            valid_values = cbf_values[~np.isnan(cbf_values)]
            if len(valid_values) > 0:
                in_range = np.sum((valid_values >= 0) & (valid_values <= 100))
                total_valid = len(valid_values)
                percentage = (in_range / total_valid) * 100
                print(f"{cbf_name} (无松弛变量): {percentage:.1f}% 的值在 [0, 100] 范围内")

def main():
    parser = argparse.ArgumentParser(description="分析CBF数据文件中的参数")
    parser.add_argument("file", help="要分析的JSON数据文件路径")
    parser.add_argument("--extract", nargs="+", 
                       help="要提取的特定参数 (例如: battery position velocity)")
    parser.add_argument("--cbf", nargs="+",
                       help="要分析的特定CBF (例如: cvtCBF energyCBF)")
    
    args = parser.parse_args()
    
    # 加载数据
    data = load_data(args.file)
    
    # 提取参数
    params = extract_parameters(data)
    
    # 如果指定了要提取的特定参数，则只显示这些参数
    if args.extract:
        print("=== 提取的参数 ===")
        for param in args.extract:
            if param == "battery":
                print("电池电量 (mV):", params["battery"])
            elif param == "position":
                print("位置:", params["position"])
            elif param == "velocity":
                print("速度:", params["velocity"])
    else:
        # 默认分析所有参数
        analyze_parameters(params)

if __name__ == "__main__":
    main()