# 自定义消息：描述一个学生的基本信息
# 语法规则：数据类型 + 字段名，一行一个字段
# 支持的类型：bool byte char float32 float64 int8 uint8 int16 uint16 int32 uint32 int64 uint64 string
#            以及上面类型的定长数组 int32[5] 和变长数组 int32[]

string   name        # 姓名
int32    age         # 年龄
float32  score       # 成绩
string[] courses     # 选修的课程列表（变长数组）
