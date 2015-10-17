import bluetooth

nearby_devices = bluetooth.discover_devices(duration=2, lookup_names=True)
for addr, name in nearby_devices:
    print("  %s - %s" % (addr, name))
