# Operating ifm cameras behind a NAT router

If you are planning to operate a ifm 3D camera behind a [NAT router](https://en.wikipedia.org/wiki/Network_address_translation) you have to consider a few things. The cameras need different ports to fully operate.

| Port  | Description  |
|-------|--------------|
|  80   | The XML-RPC communication port |
|  50010| This port is used to transfer image and result data also known a PCIC |
|  8080 | To perform a software update this port is needed |
|  55555| In case you have a PDS enabled device this port is also needed |
|  22| In case you are an OEM and running software on the device this is the SSH port |

## On the router

Typically a NAT router have different Ethernet Ports one of them is called WAN (Wide area network) this is connected to the internet or to a unknown network we want to protect the inner network from. The other ports are the internal network. We do assume the internal network is a ``192.168.0/24`` network with a gateway at ``192.168.0.1`` the router serves as a DHCP server starting from the IP ``192.168.0.100`` This will allow us to plug-in our camera with its default IP. The WAN port of the router is configured as a DHCP client. It will receive its IP by another router in the outer network. In our example the external IP of our router is: ``172.19.124.213``

| Port  | Forwarded port on the router  |
|-------|--------------|
|  80   | 5080 |
|  50010| 5010 |
|  8080 | 5081 |
|  55555| 5055 |
|  22 | 5022 |

The table above shows the forwarded ports we are using in our example.

### Port forwarding

To allow a communication from the outer network to the inner network the ports described above have to be forwarded to the camera on the local network. In our example this is the IP ``192.168.0.69``. On most routers there is a NAT tab on the network configuration where you can assign the forwarding. Depending on your router the order maybe important.

## On the camera

The factory default gateway of the camera is configured to ``192.168.0.201`` what needs to be changed to ``192.168.0.1``. To achieve this a computer with the ifm3d tools installed attached to the internal network will do the job:

```json
{
  "ifm3d": {
    "Net": {
      "NetworkSpeed": "0",
      "StaticIPv4Address": "192.168.0.69",
      "StaticIPv4Gateway": "192.168.0.1",
      "StaticIPv4SubNetMask": "255.255.255.0",
      "UseDHCP": "false"
    }
  }
}
```

Store the above JSON snippet into a file ``config.json`` and send this to the camera. If you have a different network set-up please adjust accordingly.

## ifm3d command line tools

To access the camera behind a NAT router you have to inject both the IP and the changed ports to the tooling.

```
$ ifm3d --ip=172.19.124.213 --xmlrpc-port=5080 --pcic-port=5010 ls
```

For the software update the swupdate port needs specified as-well.

```
$ ifm3d --ip=172.19.124.213 --xmlrpc-port=5080 --swu-port=5081 swupdate < latest-fw.swu
```

## Troubleshooting

When everything is set-up in a correct way but you are still unable to communicate with the camera behind the NAT router there are some pitfalls you have to consider.

### Proxy

Depending on your network set-up you may have to deal with a proxy server and it is worth to check the following environment variables: ``$http_proxy``, ``$https_proxy`` and ``$no_proxy``. If you run a proxy the recommendation is to have the external router IP added to the ``no_proxy`` environment variable.

```
export no_proxy="localhost,127.0.0.1,.ifm,192.168.0.69,172.19.124.213"
```

#### Port forwarding

To test if the port forwarding is working the simplistic test is to start a web browser and put the external address of the router in the address field [http://172.19.124.213:5080/](http://172.19.124.213:5080/). If the forwarding works an website with licensing information is shown.

If the camera is in recovery the recovery webservice is shown when using the recovery port [http://172.19.124.213:5081/](http://172.19.124.213:5081/).

The PCIC port can be tested with the telnet command:

```
$ telnet 172.19.124.213 5010
```
