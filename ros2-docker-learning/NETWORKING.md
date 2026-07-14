# Docker / ROS 2 通信ガイド

## 最初に覚える3種類

|通信|意味|設定例|
|---|---|---|
|同じCompose内|Container同士の通信|相手のサービス名を使う|
|PCからContainer|ブラウザなどから接続|`ports`を設定する|
|外部PC・ロボット|LANを越えて接続|ホストIP、ファイアウォール、ROS 2通信設定を確認|

## 同じCompose内

```yaml
services:
  server:
    image: my-server
  client:
    image: my-client
```

`client`から通常のWeb/TCPサーバーへ接続する場合、接続先は`server:8000`のようにサービス名を使えます。

## PCのブラウザからContainerへ

```yaml
ports:
  - "127.0.0.1:6080:6080"
```

- 左側：PC側のポート
- 右側：Container側のポート

この例ではPCから`http://127.0.0.1:6080`へ接続します。

## ROS 2の外部通信

最初は同じCompose内で通信を確認してください。LAN上の実機と通信するときは、UbuntuネイティブまたはJetsonでの確認を推奨します。

```yaml
services:
  robot_system:
    network_mode: host
    environment:
      ROS_DOMAIN_ID: "42"
```

`network_mode: host`、DDS、multicast、Discovery Serverなどは発展項目です。基本操作を理解した後に`index.html`のSTEP 12以降を参照してください。
