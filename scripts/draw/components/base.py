class BaseComponent:
    def update(self, num):
        """
        Update the component for frame `num`.

        Returns:
            list: A list of matplotlib artists that were modified during this update.
                  Required for blitting optimization in FuncAnimation.
        """
        return []
