# Demo Image Instructions (WIP)

The [compose file](docker-compose.yml) includes an additional service (`hw-demo`) for running on the physical robot.

This service extends the `demo` service by adding necessary configuration for interacting with ChonkUR's hardware (mostly realtime priority stuff.)

To build and run the `hw-demo` service:

```bash
# Compile the image
docker compose build hw

# Start it
docker compose up hw -d

# Connect to the console shell
docker compose exec hw bash
```
