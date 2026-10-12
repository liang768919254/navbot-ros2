# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
import pytest


# ─────────────────────────────────────────────────────────────────────────────
# 本项目【有意】豁免的 docstring 规则。每一条都有具体理由，不是"懒得改"：
#
#   D400 / D415  首行必须以 . ? ! 结尾
#                → 本项目 docstring 用中文，句末是「。」；pydocstyle 只认 ASCII 句号，
#                  属于规则与语言不兼容，无法自然满足（硬改会把中文注释变丑）
#   D403         首词必须首字母大写
#                → 有些 docstring 以代码标识符开头（readline / parse_line），
#                  标识符必须保持小写，改了就变成"写错的函数名"
#   D406/D407/D413  section 格式
#                → 本项目用 Google 风格（`Returns:`）；pydocstyle 默认按
#                  numpy 风格（`Returns` + 虚线）解析，二者互斥，只能选一个
#
# 判据：**能改的真改（D202/D213/D301 等都改掉了），不兼容的明确豁免并写清理由。**
# "有意豁免"和"无视报错"是两件事 —— 前者要留证据，后者是欠债。
# ─────────────────────────────────────────────────────────────────────────────
_IGNORE = 'D400,D415,D403,D406,D407,D413'


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test', '--add-ignore', _IGNORE])
    assert rc == 0, 'Found code style errors / warnings'
