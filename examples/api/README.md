# Simple API "wrapper" around GbcClient

This is a simple API "wrapper" around the GbcClient class. It is intended to be used as a starting point for building
your own application API, going beyond the simple examples.

The GbcClient provides an effect-driven interface to the GBC real-time control system. It allows you to register effects
that are triggered by status updates from the GBC. This is ideal for event-driven applications.

If your application is more procedural and requires an imperative coding style, you may find this simple API example
useful.