# testmode(オドメトリ、BMI270IMUデータとカルマンフィルタを用いた速度推定)

・M5stackIMUPro(BMI270) を用いています。
・タイマー割り込みで、IMUからのデータ取得と予測を行っています
・割り込み周期dtの単位はmsです
・testMode.cpp の estimator.init()でKFで用いるパラメータ設定をしています