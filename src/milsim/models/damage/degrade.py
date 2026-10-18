"""毁伤→能力降级（三杀模型，§5.17.11 草案 v0.2）。

装甲兵的"三杀"分类直接给出三个降级通道：**机动杀**（M-kill，Mover 减速）、
**火力杀**（F-kill，Weapon 拒发/耗弹翻倍）、**探测杀**（S-kill，雷达灵敏度
折扣或关机）。完好度跌破阶梯阈值后按档降级——**阶梯而不是线性**：真实装备
的降级是分级的（轻损/重损/瘫痪），线性缩放会把"断了一条履带"稀释成
"慢了 3%"这种没法裁决的中间态，而且每档一个确定数值，测试才有逐位锚点。

**档位是完好度的纯函数**——不缓存、不落账，``set_damage`` 之后下一拍自动
生效。配置挂在**平台属性**上（§5.6），与 ``max_health``/``armor`` 同一张表：
写进 ``platform_type`` 就是"按类统一设置"（沿继承链下发到所有实例），写在
``platform`` 实例块就地覆盖。**未配置 = 不降级**，直到完好度归 0（现状）。

适用判据是"多命中存活"（代偿裕度 = max_health / 典型单发伤害 ≥ 2~3）：
船、装甲车自动入选；导弹一发即死，存活状态集合只有两个点，**装配期禁配**
（见 mover 侧 ``DEGRADE_ALLOWED`` 闸门）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from ...errors import ConfigurationError

# ---------------------------------------------------------------------------
# 配置键与词汇
# ---------------------------------------------------------------------------

#: 降级配置占用的平台属性键。导弹类 mover 用它做"误配即报错"的闸门。
DEGRADE_KEYS: tuple[str, ...] = (
    "degradation",
    "degradation_thresholds",
    "mobility_scale",
    "firepower_tiers",
    "sensor_penalty_db",
)

_FIREPOWER_WORDS = ("ok", "halved", "down")
_OFF_WORDS = ("off", "关机")


def _as_float(value: Any, key: str, origin: str) -> float:
    """配置值收成 float。想定数值行给的是 float/int；字符串数字也收。"""
    if isinstance(value, bool):
        raise ConfigurationError(f"{origin}: {key} 的 {value!r} 不是数")
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            pass
    raise ConfigurationError(f"{origin}: {key} 的 {value!r} 不是数")


def _as_seq(value: Any) -> tuple[Any, ...]:
    """标量收成单项元组，序列原样；其余类型报错由调用方兜。"""
    if isinstance(value, (tuple, list)):
        return tuple(value)
    return (value,)


# ---------------------------------------------------------------------------
# 档位表
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class DegradationProfile:
    """一张降级阶梯表。

    ``L = len(mobility)`` 是档数（≥ 2）；``thresholds`` 有 L−1 个，严格
    降序、均在 (0,1)。完好度 r 落档：``r > thresholds[0]`` → 档 0（完好），
    ``thresholds[i] < r ≤ thresholds[i−1]`` → 档 i，依此类推；r ≤ 最后一个
    阈值 → 最后一档（瘫痪）。r = 0 是**摧毁**，不在本表语义内（调用方先挡）。

    三通道同长同档：**每档一行**，机动是速度比例（(0,1]）、火力是样式
    （``ok``/``halved``/``down``）、探测是灵敏度折扣（dB，≥0；``None`` =
    关机）。
    """

    name: str
    thresholds: tuple[float, ...]
    mobility: tuple[float, ...]
    firepower: tuple[str, ...]
    sensor_db: tuple[float | None, ...]

    def tier(self, intact: float) -> int:
        """完好度 → 档号。``intact ≤ 0``（摧毁）返回最后一档。"""
        if intact <= 0.0:
            return len(self.thresholds)
        for i, t in enumerate(self.thresholds):
            if intact > t:
                return i
        return len(self.thresholds)

    def mobility_scale(self, intact: float) -> float:
        """机动档比例（速度上限乘数）。"""
        return self.mobility[self.tier(intact)]

    def firepower_mode(self, intact: float) -> str:
        """火力档样式：``ok`` / ``halved``（一发耗 2 发弹）/ ``down``（拒发）。"""
        return self.firepower[self.tier(intact)]

    def sensor_factor(self, intact: float) -> float | None:
        """探测档的**线性**增益因子；``None`` = 关机（恒不探测）。

        dB 折扣只进信号账（``single_pulse_snr`` 的 SNR 上乘线性因子），
        干扰/假目标/Pd 全部自然跟随——S/(N+J) 里 S 缩小、J 不动，
        代数上恰好是"灵敏度变差"这一件事，不用另记账。
        """
        db = self.sensor_db[self.tier(intact)]
        if db is None:
            return None
        if db == 0.0:
            return 1.0
        return 10.0 ** (-db / 10.0)


#: 内置档。``armored`` = 装甲车辆三杀经典口径；``naval`` = 舰艇（血厚、
#: 分系统逐步失效，火力半残后仍能打）。
BUILTIN_PROFILES: Mapping[str, DegradationProfile] = {
    "armored": DegradationProfile(
        name="armored",
        thresholds=(0.5, 0.2),
        mobility=(1.0, 0.6, 0.25),
        firepower=("ok", "halved", "down"),
        sensor_db=(0.0, 6.0, None),
    ),
    "naval": DegradationProfile(
        name="naval",
        thresholds=(0.6, 0.3),
        mobility=(1.0, 0.75, 0.5),
        firepower=("ok", "halved", "halved"),
        sensor_db=(0.0, 4.0, 12.0),
    ),
}


# ---------------------------------------------------------------------------
# 解析与校验（装配期一次）
# ---------------------------------------------------------------------------


def _validate_profile(
    thresholds: tuple[float, ...],
    mobility: tuple[float, ...],
    firepower: tuple[str, ...],
    sensor_db: tuple[float | None, ...],
    origin: str,
) -> None:
    """四张表的长度与取值范围互相咬合，错即装配报错。"""
    n = len(mobility)
    if n < 2:
        raise ConfigurationError(f"{origin}: 档数至少 2（ mobility 有 {n} 档）")
    if not (len(thresholds) == n - 1 and len(firepower) == len(sensor_db) == n):
        raise ConfigurationError(
            f"{origin}: 档数不咬合——mobility {n} 档时 thresholds 应为 "
            f"{n - 1} 个、firepower/sensor 各 {n} 个"
            f"（来了 thresholds {len(thresholds)}、firepower {len(firepower)}、"
            f"sensor {len(sensor_db)}）"
        )
    for i, t in enumerate(thresholds):
        if not 0.0 < t < 1.0:
            raise ConfigurationError(
                f"{origin}: 阈值 {t:g} 必须落在开区间 (0,1)"
            )
        if i > 0 and t >= thresholds[i - 1]:
            raise ConfigurationError(
                f"{origin}: 阈值必须严格降序（{thresholds}），"
                "完好度从 1 往 0 掉，先过 thresholds[0] 再过 thresholds[1]"
            )
    for m in mobility:
        if not 0.0 < m <= 1.0:
            raise ConfigurationError(
                f"{origin}: 机动比例 {m:g} 必须落在 (0,1]——0 归机动杀"
                "末档的语义由档位表达，不允许直接乘 0（那等于摧毁）"
            )
    for f in firepower:
        if f not in _FIREPOWER_WORDS:
            raise ConfigurationError(
                f"{origin}: 火力样式 {f!r} 只认 {'/'.join(_FIREPOWER_WORDS)}"
            )
    for db in sensor_db:
        if db is not None and db < 0.0:
            raise ConfigurationError(
                f"{origin}: 探测折扣 {db:g} dB 必须非负——负数是增强，"
                "挨打不该越打越灵敏"
            )


def _apply_overrides(
    base: DegradationProfile,
    params: Mapping[str, Any],
    origin: str,
) -> DegradationProfile:
    """非空的覆盖键替换对应通道，其余沿用基础档。"""
    thresholds = base.thresholds
    mobility = base.mobility
    firepower = base.firepower
    sensor_db = base.sensor_db

    raw = params.get("degradation_thresholds")
    if raw not in (None, ()):
        thresholds = tuple(
            _as_float(v, "degradation_thresholds", origin)
            for v in _as_seq(raw)
        )
    raw = params.get("mobility_scale")
    if raw not in (None, ()):
        mobility = tuple(
            _as_float(v, "mobility_scale", origin) for v in _as_seq(raw)
        )
    raw = params.get("firepower_tiers")
    if raw not in (None, ()):
        seq = _as_seq(raw)
        firepower = tuple(
            v if isinstance(v, str) else str(v) for v in seq
        )
    raw = params.get("sensor_penalty_db")
    if raw not in (None, ()):
        seq = _as_seq(raw)
        parsed: list[float | None] = []
        for v in seq:
            if isinstance(v, str) and v.strip().lower() in _OFF_WORDS:
                parsed.append(None)
            else:
                parsed.append(_as_float(v, "sensor_penalty_db", origin))
        sensor_db = tuple(parsed)

    _validate_profile(thresholds, mobility, firepower, sensor_db, origin)
    return DegradationProfile(
        name=base.name,
        thresholds=thresholds,
        mobility=mobility,
        firepower=firepower,
        sensor_db=sensor_db,
    )


def resolve(params: Mapping[str, Any], *, origin: str) -> DegradationProfile | None:
    """从平台属性表解析降级配置。**未配置返回 None**（调用方走全额）。

    三种写法：

    - 什么都不写（或缺省）→ ``None``——不降级，直到完全损毁（现状）；
    - ``degradation off`` → 显式关——**实例用它覆盖类上的档**；
    - ``degradation armored``（内置档名）→ 内置档克隆，四个覆盖键
      （``degradation_thresholds`` / ``mobility_scale`` / ``firepower_tiers``
      / ``sensor_penalty_db``）非空即替换对应通道；
    - ``degradation custom`` → 自定义：``mobility_scale`` 与
      ``degradation_thresholds`` 必写，火力缺省全 ``ok``、探测缺省全不折。
    """
    raw = params.get("degradation")
    if raw is None or raw == "":
        overridden = any(params.get(k) not in (None, ()) for k in DEGRADE_KEYS[1:])
        if overridden:
            raise ConfigurationError(
                f"{origin}: 写了降级覆盖键却没写 degradation——"
                f"先给基础档（内置名或 custom），{DEGRADE_KEYS[1:]}"
            )
        return None
    if not isinstance(raw, str):
        raise ConfigurationError(f"{origin}: degradation 应是档名（字符串），来了 {raw!r}")
    name = raw.strip().lower()

    overridden = any(params.get(k) not in (None, ()) for k in DEGRADE_KEYS[1:])
    if name in _OFF_WORDS:
        if overridden:
            raise ConfigurationError(
                f"{origin}: degradation off 与覆盖键互相矛盾——关就是关"
            )
        return None

    if name == "custom":
        mobility_raw = params.get("mobility_scale")
        thresholds_raw = params.get("degradation_thresholds")
        if mobility_raw in (None, ()) or thresholds_raw in (None, ()):
            raise ConfigurationError(
                f"{origin}: degradation custom 需要 mobility_scale 与 "
                "degradation_thresholds 都写"
            )
        mobility = tuple(
            _as_float(v, "mobility_scale", origin) for v in _as_seq(mobility_raw)
        )
        thresholds = tuple(
            _as_float(v, "degradation_thresholds", origin)
            for v in _as_seq(thresholds_raw)
        )
        n = len(mobility)
        # 火力/探测没写就按 n 档"全额"补齐——custom 只想调机动时不必陪写
        firepower = tuple(
            v if isinstance(v, str) else str(v)
            for v in _as_seq(params.get("firepower_tiers") or ("ok",) * n)
        )
        sensor_seq = params.get("sensor_penalty_db") or (0.0,) * n
        parsed: list[float | None] = []
        for v in _as_seq(sensor_seq):
            if isinstance(v, str) and v.strip().lower() in _OFF_WORDS:
                parsed.append(None)
            else:
                parsed.append(_as_float(v, "sensor_penalty_db", origin))
        sensor_db = tuple(parsed)
        _validate_profile(thresholds, mobility, firepower, sensor_db, origin)
        return DegradationProfile(
            name="custom",
            thresholds=thresholds,
            mobility=mobility,
            firepower=firepower,
            sensor_db=sensor_db,
        )

    base = BUILTIN_PROFILES.get(name)
    if base is None:
        raise ConfigurationError(
            f"{origin}: 未知的降级档 {raw!r}——内置："
            f"{'、'.join(sorted(BUILTIN_PROFILES))}、custom、off"
        )
    return _apply_overrides(base, params, origin)


def configured(params: Mapping[str, Any]) -> bool:
    """平台属性里是否**真的写了**降级配置（非默认空值）。

    ``ParamSet.resolve`` 会把带默认值的键填进每个平台的属性表
    （``degradation=""``、序列 ``()``）——判"配没配"必须看值是不是空，
    不能看键在不在。
    """
    return any(params.get(k) not in (None, "", ()) for k in DEGRADE_KEYS)


def capability_of(
    params: Mapping[str, Any], intact: float, channel: str
) -> float | str | None:
    """观察口：给定平台属性与完好度，问一个通道现在的档值。

    ``channel`` ∈ ``mobility``（比例）/ ``firepower``（样式）/ ``sensor``
    （线性因子；``None`` = 该档关机）。未配置降级时：机动 1.0、火力 ``ok``、
    探测 1.0。纯函数，不碰 store——观察方自己把完好度读来。
    """
    profile = resolve(params, origin="capability_of")
    if profile is None or intact <= 0.0:
        return {"mobility": 1.0, "firepower": "ok", "sensor": 1.0}[channel]
    if channel == "mobility":
        return profile.mobility_scale(intact)
    if channel == "firepower":
        return profile.firepower_mode(intact)
    if channel == "sensor":
        return profile.sensor_factor(intact)
    raise ConfigurationError(f"capability_of: 未知通道 {channel!r}")


__all__ = [
    "DEGRADE_KEYS",
    "BUILTIN_PROFILES",
    "DegradationProfile",
    "resolve",
    "capability_of",
    "configured",
]
