# Query the PCIC ports of the heads

To get a list of all PCIC ports use the `jq` command to filter them

```
$ ifm3d  dump | jq '.. | .pcicTCPPort? // empty'
```