# atoms3-reptile-cage

ヒョウモントカゲモドキ用スマートケージの自動制御ファームウェア(M5Stack AtomS3)。

## ハードウェア

- M5Stack AtomS3
- SwitchBot プラグミニ ×1 (UVBライトの電源制御用, BLE)
- SwitchBot 防水温湿度計 (パッシブBLEスキャンで取得, 現状はコード上で無効化中)

## ビルド環境

PlatformIOで2つの環境を提供する。

| env | 用途 | ビルドコマンド |
|---|---|---|
| `m5stack-atoms3` (デフォルト) | 本番の自動制御ファームウェア (`src/automation/main.cpp`) | `pio run -e m5stack-atoms3` |
| `inspection` | 配線・機器疎通確認用の点検ツール (`src/inspection/main.cpp`) | `pio run -e inspection` |

書き込み例:

```sh
pio run -e m5stack-atoms3 --target upload --upload-port /dev/cu.usbmodemXXXX
```

## セットアップ: Wi-Fi認証情報

NTP時刻同期にWi-Fi接続を使うため、認証情報を別ファイルに分離している(ソースコードにはハードコードしない)。

```sh
cp include/wifi_config.h.example include/wifi_config.h
```

`include/wifi_config.h` に実際のSSID/パスワードを設定する。このファイルは `.gitignore` 対象。

## 制御ロジック概要 (`m5stack-atoms3`)

- **UVBライト**: 実時刻(JST)基準で 7:00-18:59 ON / 19:00-6:59 OFF を切り替える。
- **時刻同期**: 起動時と1時間ごとにWi-Fi接続→NTP同期→切断を行う(BLEとの無線干渉を避けるため、同期時以外はWi-Fiを完全にOFFにする)。
- **リトライ**: Wi-Fi接続/NTP同期、UVBプラグのON/OFF切替が失敗した場合、指数バックオフで再試行間隔を伸ばしていく(成功時はリセット)。
- **BtnA**: 押すと直近で同期した現在時刻をログに表示する。

ヒーター・ミストシステム・温湿度計の制御コードは、ライトのみ制御への切替に伴い `#if 0` で無効化して残している(将来再度有効化する場合はソース内の該当コメントを参照)。

## 点検ツール (`inspection`)

起動時にUVBプラグへの疎通確認(状態読み取り→ON→2秒待機→OFF)を自動実行する。AtomS3のBtnAで再実行できる。
