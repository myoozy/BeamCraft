package me.mzy.beamcraft.client.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import net.minecraft.world.World;

class AsyncPhysicsSchedulerTest {

    @Test
    @Timeout(5)
    void keepsOneStepInFlightAndPublishesOnlyAtTickBarrier() throws Exception {
        FakePhysicsWorld world = new FakePhysicsWorld(60.0, false);
        try (AsyncPhysicsScheduler scheduler = new AsyncPhysicsScheduler(world, 1)) {
            scheduler.startStep(null, 0.05);
            assertTrue(world.started.await(1, TimeUnit.SECONDS));
            assertThrows(IllegalStateException.class, () -> scheduler.startStep(null, 0.05));

            world.release.countDown();
            assertTrue(world.finished.await(1, TimeUnit.SECONDS));
            assertEquals(0, world.commits.get());

            AsyncPhysicsScheduler.Completion completion = scheduler.finishPreviousStep();
            assertNotNull(completion);
            assertEquals(1, world.commits.get());
            assertTrue(completion.overBudget());
            assertEquals(60.0, completion.timings()[0]);
        }
    }

    @Test
    @Timeout(5)
    void workerFailureStopsLaterSteps() throws Exception {
        FakePhysicsWorld world = new FakePhysicsWorld(1.0, true);
        try (AsyncPhysicsScheduler scheduler = new AsyncPhysicsScheduler(world, 1)) {
            scheduler.startStep(null, 0.05);
            assertTrue(world.started.await(1, TimeUnit.SECONDS));
            world.release.countDown();

            AsyncPhysicsScheduler.Completion completion = scheduler.finishPreviousStep();
            assertNotNull(completion.failure());
            assertFalse(completion.overBudget());

            scheduler.startStep(null, 0.05);
            assertEquals(1, world.prepares.get());
        }
    }

    private static final class FakePhysicsWorld extends PhysicsWorld {
        private final double totalMs;
        private final boolean fail;
        private final CountDownLatch started = new CountDownLatch(1);
        private final CountDownLatch release = new CountDownLatch(1);
        private final CountDownLatch finished = new CountDownLatch(1);
        private final AtomicInteger prepares = new AtomicInteger();
        private final AtomicInteger commits = new AtomicInteger();

        private FakePhysicsWorld(double totalMs, boolean fail) {
            this.totalMs = totalMs;
            this.fail = fail;
        }

        @Override
        public PreparedStep prepareStep(World world, double dt) {
            prepares.incrementAndGet();
            return new PreparedStep(List.of(), List.of(), dt, 100, System.nanoTime(), 0.0);
        }

        @Override
        public StepResult simulatePreparedStep(PreparedStep preparedStep) {
            started.countDown();
            try {
                if (!release.await(1, TimeUnit.SECONDS)) {
                    throw new IllegalStateException("test step was not released");
                }
            } catch (InterruptedException exception) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException(exception);
            }
            if (fail) {
                throw new IllegalStateException("expected worker failure");
            }
            finished.countDown();
            return new StepResult(preparedStep, new double[9], System.nanoTime());
        }

        @Override
        public double[] commitPreparedStep(StepResult result) {
            commits.incrementAndGet();
            result.timings()[0] = totalMs;
            return result.timings();
        }
    }
}
