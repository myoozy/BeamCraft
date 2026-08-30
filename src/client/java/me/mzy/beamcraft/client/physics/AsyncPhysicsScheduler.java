package me.mzy.beamcraft.client.physics;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.ForkJoinPool;
import java.util.concurrent.ForkJoinWorkerThread;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import net.minecraft.world.World;

/**
 * Keeps exactly one physics step in flight. The client thread prepares and
 * commits steps; only {@link PhysicsWorld#simulatePreparedStep} runs in the
 * dedicated pool.
 */
public final class AsyncPhysicsScheduler implements AutoCloseable {
    public static final double TICK_BUDGET_MS = 50.0;

    private static final AtomicInteger WORKER_IDS = new AtomicInteger();

    private final PhysicsWorld physicsWorld;
    private final ForkJoinPool physicsPool;

    private Future<PhysicsWorld.StepResult> inFlight;
    private Throwable failure;
    private boolean closed;

    public AsyncPhysicsScheduler(PhysicsWorld physicsWorld) {
        this(physicsWorld, defaultParallelism());
    }

    AsyncPhysicsScheduler(PhysicsWorld physicsWorld, int parallelism) {
        this.physicsWorld = physicsWorld;
        this.physicsPool = new ForkJoinPool(
                Math.max(1, parallelism),
                AsyncPhysicsScheduler::createWorker,
                null,
                false
        );
    }

    /**
     * Waits for and publishes the preceding tick's step. Normally the future
     * is already complete; when physics exceeds its 50 ms budget this is the
     * deliberate game-tick barrier.
     */
    public Completion finishPreviousStep() {
        Future<PhysicsWorld.StepResult> pending = inFlight;
        if (pending == null) {
            return null;
        }

        long waitStartedNanos = System.nanoTime();
        try {
            PhysicsWorld.StepResult result = pending.get();
            double waitMs = (System.nanoTime() - waitStartedNanos) / 1_000_000.0;
            double[] timings = physicsWorld.commitPreparedStep(result);
            return new Completion(timings, waitMs, timings[0] > TICK_BUDGET_MS, null);
        } catch (InterruptedException exception) {
            Thread.currentThread().interrupt();
            failure = exception;
            return new Completion(null, 0.0, false, exception);
        } catch (ExecutionException exception) {
            failure = exception.getCause() != null ? exception.getCause() : exception;
            return new Completion(null, 0.0, false, failure);
        } finally {
            inFlight = null;
        }
    }

    /**
     * Captures Minecraft state on the caller thread and starts one asynchronous
     * fixed-delta physics step.
     */
    public void startStep(World world, double dt) {
        if (closed || failure != null) {
            return;
        }
        if (inFlight != null) {
            throw new IllegalStateException("A physics step is already in flight");
        }

        PhysicsWorld.PreparedStep preparedStep = physicsWorld.prepareStep(world, dt);
        inFlight = physicsPool.submit(() -> physicsWorld.simulatePreparedStep(preparedStep));
    }

    public Throwable failure() {
        return failure;
    }

    public int parallelism() {
        return physicsPool.getParallelism();
    }

    @Override
    public void close() {
        if (closed) {
            return;
        }
        closed = true;

        Future<PhysicsWorld.StepResult> pending = inFlight;
        if (pending != null) {
            try {
                pending.get();
            } catch (InterruptedException exception) {
                Thread.currentThread().interrupt();
            } catch (ExecutionException ignored) {
                // The client is already stopping; the tick path reports live failures.
            } finally {
                inFlight = null;
            }
        }

        physicsPool.shutdown();
        try {
            if (!physicsPool.awaitTermination(5, TimeUnit.SECONDS)) {
                physicsPool.shutdownNow();
            }
        } catch (InterruptedException exception) {
            physicsPool.shutdownNow();
            Thread.currentThread().interrupt();
        }
    }

    private static int defaultParallelism() {
        return Math.max(1, Runtime.getRuntime().availableProcessors() - 2);
    }

    private static ForkJoinWorkerThread createWorker(ForkJoinPool pool) {
        ForkJoinWorkerThread worker = ForkJoinPool.defaultForkJoinWorkerThreadFactory.newThread(pool);
        worker.setName("BeamCraft-Physics-" + WORKER_IDS.incrementAndGet());
        worker.setPriority(Math.max(Thread.MIN_PRIORITY, Thread.NORM_PRIORITY - 1));
        return worker;
    }

    public record Completion(double[] timings, double waitMs, boolean overBudget, Throwable failure) {
    }
}
