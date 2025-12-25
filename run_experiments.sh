#!/bin/bash

# 检查是否提供了地图名和agents数量
if [ $# -lt 2 ]; then
    echo "Usage: $0 <map_name> <number_of_agents1> [<number_of_agents2> ...]"
    echo "Example: $0 random-32-32-20 10 20 30"
    exit 1
fi

# 获取地图名和agents数量
MAP_NAME=$1
shift
AGENTS_LIST=("$@")

# 设置基础路径
INSTANCE_BASE="../instances/test/${MAP_NAME}"
OUTPUT_BASE="../experiment/${MAP_NAME}"
SOLVER="PIBT"

# 遍历每个agents数量
for AGENTS in "${AGENTS_LIST[@]}"; do
    # 创建输出目录
    OUTPUT_DIR="${OUTPUT_BASE}/${AGENTS}_swap_escape"
    mkdir -p $OUTPUT_DIR

    # 创建CSV文件并写入表头
    echo "Instance,Solved,preprocessing_comp_time,comp_time,soc,makespan" > "$OUTPUT_DIR/summary.csv"

    # 运行25个算例
    for i in {1..25}
    do
        # 构建文件路径
        INPUT_FILE="${INSTANCE_BASE}/${AGENTS}/${MAP_NAME}-${AGENTS}-$i.txt"
        OUTPUT_FILE="${OUTPUT_DIR}/result_${MAP_NAME}-${AGENTS}-$i.txt"
        
        echo "Running instance $i of 25 for ${AGENTS} agents on map ${MAP_NAME}..."
        
        # 运行程序
        ./mapf -i $INPUT_FILE -s $SOLVER -o $OUTPUT_FILE -v
        
        # 检查运行状态
        if [ $? -eq 0 ]; then
            # 使用awk提取所有需要的数据
            RESULT=$(awk '
                /^solved=/ {solved=$0}
                /^preprocessing_comp_time=/ {preproc=$0}
                /^comp_time=/ {comp=$0}
                /^soc=/ && !/^lb_soc=/ {soc=$0}
                /^makespan=/ && !/^lb_makespan=/ {makespan=$0}
                END {
                    split(solved, s, "=");
                    split(preproc, p, "=");
                    split(comp, c, "=");
                    split(soc, sc, "=");
                    split(makespan, m, "=");
                    printf "%d,%d,%d,%d,%d", s[2], p[2], c[2], sc[2], m[2]
                }' "$OUTPUT_FILE")
            
            # 构建完整的一行数据
            INSTANCE="${MAP_NAME}-${AGENTS}-$i"
            echo "$INSTANCE,$RESULT" >> "$OUTPUT_DIR/summary.csv"
            
            echo "Instance $i for ${AGENTS} agents completed successfully"
        else
            echo "Error running instance $i for ${AGENTS} agents"
            # 记录失败情况
            INSTANCE="${MAP_NAME}-${AGENTS}-$i"
            echo "$INSTANCE,0,,,," >> "$OUTPUT_DIR/summary.csv"
        fi
        
        echo "----------------------------------------"
    done

    echo "All instances for ${AGENTS} agents completed. Results saved in ${OUTPUT_DIR}/summary.csv"
done

echo "All experiments completed."