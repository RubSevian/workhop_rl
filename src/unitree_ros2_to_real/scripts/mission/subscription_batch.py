"""Bounded nonblocking drain of one subscription, inside its exclusive callback.

Use the same Jazzy take_message interface as rclpy's executor. Preserve every
received measurement for history, but process current status only once/batch.
No waiting for future samples, no timestamp rewriting.
"""


def take_batch(subscription, first, limit):
    if type(limit) is not int or not 1 <= limit <= 400:
        raise ValueError('invalid subscription batch limit')
    messages = [first]
    if subscription is not None:
        with subscription.handle:
            for _ in range(limit - 1):
                item = subscription.handle.take_message(subscription.msg_type, subscription.raw)
                if item is None:
                    break
                messages.append(item[0])
    return messages
