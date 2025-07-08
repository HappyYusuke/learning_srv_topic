# learning_srv_topic

デッドロックする例

```
ros2 run learning_srv_topic pub_topic
ros2 run learning_srv_topic bad_srv_server
```

</br>

うまく動作する例

```
ros2 run learning_srv_topic pub_topic
ros2 run learning_srv_topic good_srv_server
```
