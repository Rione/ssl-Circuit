# LT3751 フライバックコンバータ設計計算ツール
# 設定ファイル (YAML) から入力値を読み込み、各種パラメータを計算して出力する。

# --実行方法--
# source .venv/bin/activate　で仮想環境を有効化
# pip install pyyaml で必要なライブラリをインストール
# python main.py config.yaml で実行
# deactivate　で仮想環境を無効化


import yaml
import sys

# ===== 関数定義 =====
def check_Nratio_setting(N, Vout, Vtrans):
    """
    巻数比 N が適切かチェックする。
    N は Vout / Vtrans より小さくなければならない。
    """
    if N > Vout / Vtrans:
        raise ValueError("巻数比 N は Vout / Vtrans より大きくなければなりません。")

def check_Ipk_setting(Lpri, Ipk, Vout, N):
    """
    ピーク電流 IPK が適切かチェックする。
    Lpri = Vout * 3us / (N * Ipk) である必要がある。
    """
    if 3.0e-6 * Vout / (N * Ipk) != Lpri:
        raise ValueError("ピーク電流 IPK の設定が不適切です。 Lpri = Vout * 3us / (N * IPK) を満たす必要があります。")

def calc_Iave_forNmos(Ipk, Vout, N, Vtrans):
    """
    Nmos の平均電流を計算する。
    Iave = Ipk * Vout / 2 / (Vout + N * Vtrans)
    """
    return Ipk * Vout / 2 / (Vout + N * Vtrans)

def calc_Vrrm_forDiode(Vout, N, Vtrans):
    """
    ダイオードの逆電圧 VRRM（最大反復逆電圧） を計算する。
    VRRM = Vout + N * Vtrans
    """
    return Vout + N * Vtrans

def calc_If_forDiode(Ipk, N):
    """
    ダイオードの必要最小電流 IF を計算する。
    IF = Ipk / N
    """
    return Ipk / 2 / N

def calc_Rbg(N, RVout, Vout, Vdiode):
    """
    バイアス抵抗 RBG を計算する。
    RBG = 0.98 * N * RVout / (Vout + Vdiode)
    """
    return 0.98 * N * RVout / (Vout + Vdiode)

def calc_Rsense(Ipk):
    """
    センス抵抗 Rsense を計算する。
    Rsense = 106e-3 / Ipk
    """
    return 106e-3 / Ipk

def calc_Psense(Rsense, Ipk, Vout, N, Vtrans):
    """
    センス抵抗の最大定格を計算する。
    Psense = Rsense * Ipk^2 * Vout / (Vout + N * Vtrans) / 3
    """
    return Rsense * Ipk**2 * Vout / (Vout + N * Vtrans) / 3

def calc_Ipk(Rsense):
    """
    ピーク電流 Ipk を計算する。
    Ipk = 106e-3 / Rsense
    """
    return 106e-3 / Rsense

def calc_Vout(N, RVout, Vdiode, Rbg):
    """
    出力電圧 Vout を計算する。
    Vout = 0.98 * N * RVout / Rbg - Vdiode
    """
    return 0.98 * N * RVout / Rbg - Vdiode

def calc_Rfbh(Vfb, Rfbl):
    """
    フィードバック抵抗 RFBH を計算する。
    Vfb * Rfbh/(Rfbh+Rfbl) = 1.16になる
    Rfbh = Rfbl * (Vfb - 1.16) 
    """
    return Rfbl * (Vfb - 1.16)

def calc_VdsMax(Vout, N, VtransMax):
    """
    最大 Vds を計算する。
    VdsMax = VtransMax + Vout / N
    """
    return VtransMax + Vout / N

# ===== メイン処理 =====

def main(config_file):
    # YAML 設定読み込み
    with open(config_file, 'r') as f:
        cfg = yaml.safe_load(f)

    # 入力値取得
    VtransMax = cfg['VtransMax']
    VtransMin = cfg['VtransMin']
    N = cfg['N']
    Lpri = cfg['Lpri']
    Vdiode = cfg['Vdiode']
    RVout = cfg['RVout']
    Rbg = cfg['Rbg']
    Rsense = cfg['Rsense']
    Vfb = cfg['Vfb']
    Rfbl = cfg['Rfbl']

    # 1. 設定値のチェック
    Ipk = calc_Ipk(Rsense)
    Vout = calc_Vout(N, RVout, Vdiode, Rbg)
    check_Nratio_setting(N, Vout, VtransMax)

    # 2. 平均電流計算
    IaveMin = calc_Iave_forNmos(Ipk, Vout, N, VtransMin)
    IaveMax = calc_Iave_forNmos(Ipk, Vout, N, VtransMax)
    VdsMax = calc_VdsMax(Vout, N, VtransMax)

    # 3. ダイオード
    VrrmMin = calc_Vrrm_forDiode(Vout, N, VtransMin)
    VrrmMax = calc_Vrrm_forDiode(Vout, N, VtransMax)
    If = calc_If_forDiode(Ipk, N)

    # 4. センス抵抗
    Psense = calc_Psense(Rsense, Ipk, Vout, N, VtransMin)   
    Rfbh = calc_Rfbh(Vfb, Rfbl)

    # 5. 結果表示
    print("=== LT3751 設計結果 ===")

    print(f"トランスの最大電圧 Vtrans (最小): {VtransMin:.1f} V")
    print(f"トランスの最大電圧 Vtrans (最大): {VtransMax:.1f} V")
    print(f"巻数比 N: {N:.0f}")
    print(f"一次側インダクタンス Lpri: {Lpri*1000000:.0f} uH")
    print(f"ダイオードの順方向電圧 Vdiode: {Vdiode:.1f} V")
    print(f"出力電圧の抵抗 RVout: {RVout/1000:.1f} kΩ")
    print(f"バイアス抵抗 RBG: {Rbg/1000:.2f} kΩ")
    print(f"センス抵抗 Rsense: {Rsense*1000:.0f} mΩ")

    print("========================")

    print(f"出力電圧 Vout: {Vout:.1f} V")
    print(f"ピーク電流 Ipk: {Ipk:.2f} A")
    print(f"平均電流 (最小): {IaveMin:.2f} A")
    print(f"平均電流 (最大): {IaveMax:.2f} A")
    print(f"最大 Vds: {VdsMax:.2f} V")
    print(f"ダイオードの逆電圧 (最小): {VrrmMin:.2f} V")
    print(f"ダイオードの逆電圧 (最大): {VrrmMax:.2f} V")
    print(f"ダイオードの平均電流: {If:.2f} A")
    print(f"センス抵抗の最大定格 Psense: {Psense:.2f} W")
    print(f"フィードバック抵抗 RFBH: {Rfbh/1000:.2f} kΩ")

    print("========================")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python main.py <config.yaml>")
        sys.exit(1)
    main(sys.argv[1])