"""框架的错误基类。

为什么要有统一基类
------------------
本框架里"可预期的错误"有六种来源：想定词法、想定语法、想定语义、参数校验、
类型定义、装配。它们分布在不同模块，但**对调用方是同一件事**：配置写错了，
请把消息打给用户看。

没有统一基类时，工具脚本要写：

.. code-block:: python

    except (LexError, ParseError, ScenarioError, ParamError, TypeDefinitionError):

每加一种校验就要回头改所有调用点，漏一个就是栈回溯。有了基类：

.. code-block:: python

    except MilsimError as exc:
        print(exc)

这条边界值得划清：**配置错误**（用户能改）与**代码缺陷**（要改框架）走
不同的出口。前者给一句人话就够，后者必须让栈回溯露出来。

为什么不直接继承 ValueError
---------------------------
`ConfigurationError` **同时**继承 `MilsimError` 与 `ValueError`——
历史上这些异常都是 ValueError，改成单一继承会让既有的
``except ValueError`` 失效。两者兼顾，新旧代码都不用改。
"""

from __future__ import annotations


class MilsimError(Exception):
    """本框架所有**可预期错误**的基类。

    捕获它意味着"处理配置问题"，不意味着"吞掉所有异常"——代码缺陷引发的
    异常（`TypeError`、`AttributeError`、`ZeroDivisionError`）不会落进来，
    这是刻意的。
    """


class ConfigurationError(MilsimError, ValueError):
    """想定或配置写错了。用户改想定就能解决，不需要改代码。"""


__all__ = ["ConfigurationError", "MilsimError"]
