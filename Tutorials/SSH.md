# SSH to Jetson Nano


Requirements:
Jetson Nano and your laptop must be on the same network (same Wi-Fi or same router via Ethernet).

---

## 1. Find Jetson Nano’s IP Address

On the Jetson Nano (using a monitor + keyboard), open a terminal and run:


```bash
ip addr show
```

Look for wlan0 (Wi-Fi).
Example output might show something like:

```bash
inet 192.168.1.42
```
So the IP is: 192.168.1.42

---

## 2. SSH into the Nano from your Laptop

If your username on the Jetson is jetson and the hostname is nano (jetson@nano), then your SSH command is:


```bash
ssh jetson@192.168.1.42
```

If successful, your prompt will change to something like:

```bash
jetson@nano:~$
```


---
