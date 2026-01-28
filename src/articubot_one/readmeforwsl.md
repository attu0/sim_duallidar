#commands to remove wsl 

```
wsl --shutdown
```
```
wsl --list --verbose
```

```
wsl --unregister Ubuntu-22.04
```

##making directory

```
mkdir -p dev_ws/src
```

```
cd dev_ws/src
```

```
git clone https://github.com/attu0/articubot_one.git
```

```
cd ..
```

```
colcon build --symlink-install
```

ros2 launch 